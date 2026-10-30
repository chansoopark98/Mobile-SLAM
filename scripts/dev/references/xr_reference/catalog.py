#!/usr/bin/env python3
"""Snapshot official xr_reference public repositories without modifying existing clones."""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/dev'))
from reference_metadata import load_reference_metadata
SOURCE = load_reference_metadata(ROOT)
CORE = set(SOURCE['core_repositories'])


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], stderr=subprocess.PIPE).decode('utf-8', 'surrogateescape').strip()


def canonical(url):
    return url.removesuffix('.git').replace('git@github.com:', 'https://github.com/').lower()


def discover(root):
    clones = {}
    for base, dirs, files in os.walk(root):
        clone_root = '.git' in dirs or '.git' in files
        candidates = [Path(base)] if clone_root else [
            Path(base) / name for name in dirs
            if (Path(base) / name).is_symlink() and (Path(base) / name / '.git').exists()
        ]
        for path in candidates:
            try:
                remote = canonical(git(path, 'remote', 'get-url', 'origin'))
                clones.setdefault(remote, path)
            except subprocess.CalledProcessError:
                pass
        if clone_root:
            dirs[:] = []
    return clones


def inventory(evidence):
    repos = []
    pages = []
    for page in range(1, 100):
        url = SOURCE['organization_api_url'] + f'?type=public&per_page=100&page={page}'
        request = urllib.request.Request(url, headers={'User-Agent': 'Mobile-SLAM-reference-catalog', 'Accept': 'application/vnd.github+json'})
        with urllib.request.urlopen(request, timeout=60) as response:
            payload = response.read()
            rows = json.loads(payload)
            (evidence / f'github-page-{page}.json').write_bytes(payload)
            pages.append({'url': url, 'count': len(rows), 'sha256': hashlib.sha256(payload).hexdigest(), 'headers': dict(response.headers)})
        if not rows:
            break
        repos.extend(rows)
    else:
        raise RuntimeError('Pagination safety limit reached; inventory is incomplete')
    if len({repo['full_name'] for repo in repos}) != len(repos):
        raise RuntimeError('Duplicate repositories across API pages; rerun inventory')
    write_json(evidence / 'github-pages.json', pages)
    write_json(evidence / 'github-repositories.json', repos)
    return repos


def inspect(repo, path, action):
    root = Path(git(path, 'rev-parse', '--show-toplevel')).resolve()
    if root != path.resolve():
        raise RuntimeError(f'Not a standalone clone: {path}')
    remote = git(path, 'remote', 'get-url', 'origin')
    if canonical(remote) != canonical(repo['clone_url']):
        raise RuntimeError(f'Remote mismatch: {remote}')
    tracked = subprocess.check_output(['git', '-C', str(path), 'ls-files', '-z']).split(b'\0')
    pointers = []
    unreadable = []
    for raw in tracked:
        if not raw:
            continue
        name = os.fsdecode(raw)
        target = path / name
        if not target.is_file() or target.is_symlink():
            continue
        try:
            size = target.stat().st_size
            if size < 2048:
                content = target.read_bytes()
                if content.startswith(b'version https://git-lfs.github.com/spec/v1\n'):
                    text = content.decode('ascii')
                    pointers.append({'path': name, 'pointer': text})
        except (OSError, UnicodeError) as exc:
            unreadable.append({'path': name, 'error': str(exc)})
    return {
        'name': repo['name'], 'url': repo['html_url'], 'description': repo['description'],
        'fork': repo['fork'], 'archived': repo['archived'], 'api_default_branch': repo['default_branch'],
        'category': 'core' if repo['name'] in CORE else ('dependency-fork' if repo['fork'] else ('example' if 'example' in repo['name'] else 'supporting')),
        'path': str(path.resolve()), 'action': action, 'remote': remote,
        'head': git(path, 'rev-parse', 'HEAD'), 'branch': git(path, 'branch', '--show-current'),
        'shallow': git(path, 'rev-parse', '--is-shallow-repository') == 'true',
        'status_porcelain': git(path, 'status', '--porcelain=v1', '--untracked-files=normal'),
        'tracked_file_count': len([p for p in tracked if p]),
        'lfs_pointer_count': len(pointers), 'lfs_pointers': pointers, 'scan_errors': unreadable,
        'lfs_scan_scope': 'tracked regular working-tree files smaller than 2048 bytes; symlinks excluded',
    }


def clone_or_inspect(repo, known, destination, evidence):
    path = known.get(canonical(repo['clone_url']), destination / repo['name'])
    action = 'reused' if canonical(repo['clone_url']) in known else 'cloned'
    try:
        if action == 'cloned':
            if path.exists():
                raise RuntimeError(f'Unverified existing destination preserved: {path}')
            env = dict(os.environ, GIT_LFS_SKIP_SMUDGE='1', GIT_TERMINAL_PROMPT='0')
            command = ['git', 'clone', '--depth', '1', '--single-branch', '--branch', repo['default_branch'], repo['clone_url'], str(path)]
            with (evidence / f"clone-{repo['name']}.log").open('wb') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True, timeout=1800)
        result = inspect(repo, path, action)
        write_json(evidence / f"repo-{repo['name']}.json", result)
        print(f"OK {repo['name']} {action} {result['head']} {path}", flush=True)
        return result
    except Exception as exc:
        result = {'name': repo['name'], 'url': repo['html_url'], 'path': str(path), 'error': str(exc)}
        write_json(evidence / f"repo-{repo['name']}.error.json", result)
        print(f"FAIL {repo['name']}: {exc}", flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('assets/references/xr_reference'))
    parser.add_argument('--evidence', type=Path, default=Path('build/xr_reference-deep-20261014/catalog'))
    parser.add_argument('--jobs', type=int, choices=range(1, 5), default=3)
    args = parser.parse_args()
    args.root.mkdir(parents=True, exist_ok=True)
    args.evidence.mkdir(parents=True, exist_ok=True)
    destination = args.root / 'official'
    destination.mkdir(exist_ok=True)
    start = datetime.datetime.now(datetime.timezone.utc).isoformat()
    repos = inventory(args.evidence)
    known = discover(args.root)
    # Core clones become available early to independent analysis lanes.
    repos.sort(key=lambda r: (r['name'] not in CORE, r['name'] != SOURCE['main_repository_name'], r['name']))
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(clone_or_inspect, repo, known, destination, args.evidence) for repo in repos]
        results = [future.result() for future in concurrent.futures.as_completed(futures)]
    failures = [result for result in results if 'error' in result]
    successful = [result for result in results if 'error' not in result]
    manifest = {
        'started_utc': start, 'finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'organization': SOURCE['organization_url'], 'repository_count': len(repos),
        'success_count': len(successful), 'failure_count': len(failures),
        'fork_count': sum(repo['fork'] for repo in repos),
        'categories': {key: sum(result['category'] == key for result in successful) for key in ['core', 'dependency-fork', 'example', 'supporting']},
        'lfs_policy': 'GIT_LFS_SKIP_SMUDGE=1 on new clones; existing checkouts are not modified; submodules are not initialized',
        'preservation': 'No fetch/reset/stash/clean or overwrite of an existing checkout',
        'repositories': sorted(results, key=lambda result: result['name']),
    }
    write_json(args.evidence / 'manifest.json', manifest)
    print(json.dumps({key: value for key, value in manifest.items() if key != 'repositories'}, indent=2), flush=True)
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
