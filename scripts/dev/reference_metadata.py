"""Read the upstream acquisition/provenance settings documented in the project README."""
import json
from pathlib import Path
import re


def load_reference_metadata(repo):
    text = (Path(repo) / 'README.md').read_text(encoding='utf-8')
    match = re.search(r'^```public-reference-metadata\n(.*?)\n```', text, re.M | re.S)
    if match is None:
        raise ValueError('README.md lacks the public-reference-metadata block')
    return json.loads(match.group(1))


def matches_reference_schema(schema, expected, repo):
    """Accept the current name or the original source name for this exact schema version."""
    if not isinstance(schema, str):
        return False
    if schema == expected:
        return True
    legacy_name = load_reference_metadata(repo)['main_repository_name']
    return schema == expected.replace('xr_reference', legacy_name)
