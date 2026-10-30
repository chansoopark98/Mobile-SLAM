#!/usr/bin/env python3
"""Mutation tests for check_vendor.py: the include-only patch check (RM-1c) and the working-tree base (R4).

Run: python3 -I -B slam_refact/engine/tools/test_check_vendor.py

The check must refuse every changed line of an "include lines only" patch that is not a single
#include with at most one comment. Positive controls (an include with a closed comment) must pass.
The two blind spots that review RM-1b/RM-5 MINOR-3 found are cases below: two block comments
around code, and a `//` comment ending in a backslash. RM-1d adds the lone carriage return (and
FF / VT) of review RM-1c MINOR-2: the file is read with newline='' so that a CR stays inside its
changed line instead of turning the code after it into an unchanged context line.
"""
import importlib.util
import os
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('check_vendor', os.path.join(HERE, 'check_vendor.py'))
check_vendor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_vendor)

ACCEPTED = [
    '+#include "a.h"',
    '+#include <a.h>',
    '-#include "a.h"',
    '+  #  include   "a.h"   ',
    '+#include "a.h" // why',
    '+#include "a.h" /* why */',
    '+#include "a.h"   /* why */   ',
    '+#include "a.h" /* a * b */',
    '+#include "a.h" /** doc **/',
    # RM-1d: the CR of a CRLF line end is not a line break inside the line
    '+#include "a.h"\r',
    '+#include "a.h" // why\r',
]

REFUSED = [
    # pre-RM-1c cases
    '+++g_count;',
    '---n;',
    '+int z = 1;',
    '+#include "a.h" int y = 0;',
    '+#include "a.h" /* unclosed',
    # RM-1c blind spot 1: the greedy block comment used to swallow the code between two comments
    '+#include "a.h" /* x */ int y = 0; /* z */',
    '+#include "a.h" /* x */ int y = 0;',
    '+#include "a.h" /* x */ /* y */ int z;',
    # RM-1c blind spot 2: a trailing backslash continues the line (or a // comment) into the next
    # line, which is code that the check never sees
    '+#include "a.h" // hides the next line \\',
    '+#include "a.h" \\',
    '+#include "a.h" /* x */ \\',
    '+// not an include \\',
    # RM-1d (review RM-1c MINOR-2): a lone CR (patch applies it as it is, a C compiler ends the line
    # there) hides code behind a comment or an include; FF and VT have no place in an include line
    '+#include "a.h" // c\rint y = 7;',
    '+#include "a.h"\rint y = 7;',
    '+#include "a.h" /* x */\rint y = 7;',
    '+#include "a.h"\r\r',
    '+#include "a.h"\f',
    '+#include "a.h" // c\vint y;',
]


def patch_text(changed):
    return ('--- a/x.cc\n+++ b/x.cc\n@@ -1,2 +1,3 @@\n context\n%s\n context2\n' % changed)


class IncludeOnlyLine(unittest.TestCase):
    def test_accepts_plain_includes_and_closed_comments(self):
        for line in ACCEPTED:
            with self.subTest(line=line):
                self.assertTrue(check_vendor.is_include_only_line(line))

    def test_refuses_everything_else(self):
        for line in REFUSED:
            with self.subTest(line=line):
                self.assertFalse(check_vendor.is_include_only_line(line))


class PatchHunks(unittest.TestCase):
    """End to end: check_patch_hunks reads a hunk and reports a failure for each refused line."""

    def run_check(self, changed):
        with tempfile.TemporaryDirectory() as tmp:
            patches = os.path.join(tmp, 'xr_source', 'patches')
            os.makedirs(patches)
            with open(os.path.join(patches, 'p.patch'), 'w') as f:
                f.write(patch_text(changed))
            old = (check_vendor.TP, check_vendor.INCLUDE_ONLY_PATCHES)
            check_vendor.TP, check_vendor.INCLUDE_ONLY_PATCHES = tmp, ('p.patch',)
            try:
                res = check_vendor.Result()
                check_vendor.check_patch_hunks(res)
                return res.failures
            finally:
                check_vendor.TP, check_vendor.INCLUDE_ONLY_PATCHES = old

    def test_accepted_lines_pass(self):
        for line in ACCEPTED:
            if line.startswith('+'):  # the hunk header above counts one added line
                with self.subTest(line=line):
                    self.assertEqual(self.run_check(line), [])

    def test_refused_lines_are_reported(self):
        for line in REFUSED:
            with self.subTest(line=line):
                self.assertEqual(len(self.run_check(line)), 1)


class WorktreeBase(unittest.TestCase):
    """R4: --upstream-base worktree hashes the clone's current files as they are (no strip, no git)."""

    BODY = b'int a;\n'

    def run_base(self, clone_bytes, row_sha=None, vendored=None, patches=()):
        with tempfile.TemporaryDirectory() as tmp:
            up = os.path.join(tmp, 'clone')
            os.makedirs(up)
            if clone_bytes is not None:
                with open(os.path.join(up, 'a.h'), 'wb') as f:
                    f.write(clone_bytes)
            sha = check_vendor.sha256_bytes
            row = dict(path='a.h', upstream_path='a.h', role='header', patches='-',
                       upstream_sha256=row_sha or sha(self.BODY),
                       vendored_sha256=vendored or sha(self.BODY))
            saved = (check_vendor.UPSTREAM_8W, check_vendor.TP, check_vendor.PATCH_ORDER)
            os.makedirs(os.path.join(tmp, 'tp', 'xr_source', 'patches'))
            check_vendor.UPSTREAM_8W, check_vendor.TP, check_vendor.PATCH_ORDER = up, os.path.join(tmp, 'tp'), []
            try:
                res = check_vendor.Result()
                check_vendor.check_xr_reference_upstream(res, [row], 'worktree')
                return res
            finally:
                check_vendor.UPSTREAM_8W, check_vendor.TP, check_vendor.PATCH_ORDER = saved

    def test_matching_clone_file_passes_without_git(self):
        res = self.run_base(self.BODY)
        self.assertEqual(res.failures, [])
        self.assertTrue(any('commit-vs-working-tree comparison skipped' in n for n in res.notes))

    def test_edited_clone_file_fails(self):
        self.assertEqual(len(self.run_base(b'int b;\n').failures), 1)

    def test_clone_file_with_copyright_line_is_not_stripped(self):
        res = self.run_base(b'// Copyright (c) 2024 reference runtime, LLC.\n' + self.BODY)
        self.assertEqual(len(res.failures), 1)

    def test_missing_clone_file_fails(self):
        self.assertEqual(len(self.run_base(None).failures), 1)

    def test_vendored_must_equal_clone_file_plus_patches(self):
        res = self.run_base(self.BODY, vendored=check_vendor.sha256_bytes(b'int c;\n'))
        self.assertEqual(len(res.failures), 1)

    def test_public_author_comment_keeps_original_source_hash(self):
        compact = check_vendor.REPO_8W.rstrip('/').rsplit('/', 1)[-1]
        original = ('// Original Author: Test Person (test@%s.com)\nint a;\n' % compact).encode()
        public = b'// Original Author: Test Person\nint a;\n'
        res = self.run_base(original, row_sha=check_vendor.sha256_bytes(original),
                            vendored=check_vendor.sha256_bytes(public))
        self.assertEqual(res.failures, [])

    def test_public_renaming_does_not_allow_source_algorithm_mutation(self):
        owner = check_vendor.REFERENCE_METADATA['copyright_owners'][0]
        original = ('// %s comparison\nint a;\n' % owner).encode()
        mutated = original.replace(b'int a;', b'int b;')
        public = b'// reference runtime comparison\nint b;\n'
        res = self.run_base(mutated, row_sha=check_vendor.sha256_bytes(original),
                            vendored=check_vendor.sha256_bytes(public))
        self.assertEqual(len(res.failures), 1)


class CopyrightPolicy(unittest.TestCase):
    def test_policy_owner_is_read_from_source_disclosure(self):
        owner = check_vendor.REFERENCE_METADATA['copyright_owners'][0]
        original = ('// Copyright (c) 2024 %s, LLC.\nint a;\n' % owner).encode()
        self.assertEqual(check_vendor.strip_policy_line(original), (b'int a;\n', True))

    def test_unrelated_third_party_notice_is_preserved(self):
        original = b'// Copyright (c) 2024 Another Library\nint a;\n'
        self.assertEqual(check_vendor.strip_policy_line(original), (original, False))


class PublicProbeSource(unittest.TestCase):
    def check_probe(self, body):
        compact = check_vendor.REPO_8W.rstrip('/').rsplit('/', 1)[-1]
        historical = ('// Probe for public %s\nint a;\n' % compact).encode()
        with tempfile.TemporaryDirectory() as tmp:
            source = os.path.join(tmp, 'probe.h')
            with open(source, 'wb') as f:
                f.write(b'// Probe for public xr_reference\n' + body)
            saved = check_vendor.REPO
            check_vendor.REPO = tmp
            try:
                row = dict(path='probe.h', upstream_path='probe.h', upstream_commit='unpublished',
                           upstream_sha256=check_vendor.sha256_bytes(historical),
                           vendored_sha256=check_vendor.sha256_bytes(
                               b'// Probe for public xr_reference\nint a;\n'), patches='-')
                result = check_vendor.Result()
                check_vendor.check_shim_upstream(result, [row])
                return result.failures
            finally:
                check_vendor.REPO = saved

    def test_comment_change_reconstructs_pinned_hash_without_git(self):
        self.assertEqual(self.check_probe(b'int a;\n'), [])

    def test_algorithm_change_cannot_reconstruct_pinned_hash(self):
        self.assertEqual(len(self.check_probe(b'int b;\n')), 1)


if __name__ == '__main__':
    unittest.main()
