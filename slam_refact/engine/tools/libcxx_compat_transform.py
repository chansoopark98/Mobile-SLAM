#!/usr/bin/env python3
"""Mechanical first stage of the libcxx_compat excerpt (third_party/libcxx_compat, WP0a).

Reads the libc++ headers of the installed emsdk 5.0.0 (LLVM 21.1.8) and writes renamed copies:
  - _LIBCPP_BEGIN/END_NAMESPACE_STD -> _SR_LIBCXX_BEGIN/END_NAMESPACE (namespace slam_refact::libcxx)
  - every other _LIBCPP_* macro     -> _SR_LIBCPP_* (defined in libcxx_compat/__config.h)
  - std::__name and std::<excerpted public name> -> ::slam_refact::libcxx::name
  - #include <__x/y.h>              -> the excerpt header, or a comment naming __config.h
Algorithm lines are not touched. The manual second stage (recorded in
third_party/libcxx_compat/PROVENANCE.md) removed iostream operators, debug-only classes and the
extern-template block of sort.h, and pinned the shuffle distribution type.

Usage: libcxx_compat_transform.py <out_dir>
The libc++ headers are read from $EMSDK/upstream/emscripten/system/lib/libcxx/include ($EMSDK
defaults to ~/emsdk), the same root check_vendor.py uses.
"""
import os, re, sys, pathlib
EMSDK = pathlib.Path(os.environ.get('EMSDK', os.path.expanduser('~/emsdk')))
X = EMSDK / 'upstream/emscripten/system/lib/libcxx/include'
OUT = pathlib.Path(sys.argv[1])
FILES = ['__algorithm/sort.h','__algorithm/nth_element.h','__algorithm/partial_sort.h','__algorithm/make_heap.h',
 '__algorithm/push_heap.h','__algorithm/pop_heap.h','__algorithm/sort_heap.h','__algorithm/sift_down.h',
 '__algorithm/min_element.h','__algorithm/shuffle.h','__random/log2.h','__random/uniform_int_distribution.h',
 '__random/generate_canonical.h','__random/uniform_real_distribution.h','__random/normal_distribution.h']
OURS = {'sort','nth_element','partial_sort','make_heap','push_heap','pop_heap','sort_heap','shuffle',
        'generate_canonical','min_element','uniform_int_distribution','uniform_real_distribution','normal_distribution'}
for f in FILES:
  t = (X/f).read_text()
  t = t.replace('_LIBCPP_BEGIN_NAMESPACE_STD', '_SR_LIBCXX_BEGIN_NAMESPACE')
  t = t.replace('_LIBCPP_END_NAMESPACE_STD', '_SR_LIBCXX_END_NAMESPACE')
  t = re.sub(r'\b_LIBCPP_', '_SR_LIBCPP_', t)
  def q(m):
    name = m.group(1)
    if name.startswith('__') or name in OURS:
      return '::slam_refact::libcxx::' + name
    return m.group(0)
  t = re.sub(r'\bstd::(\w+)', q, t)
  def inc(m):
    h = m.group(1)
    if h in FILES:
      return '#include "libcxx_compat/%s"\n' % h
    if h == '__config':
      return '#include "libcxx_compat/__config.h"\n'
    return '// [libcxx_compat/__config.h] #include <%s>\n' % h
  t = re.sub(r'^#include <(__[^>]+)>\n', inc, t, flags=re.M)
  o = OUT/f; o.parent.mkdir(parents=True, exist_ok=True); o.write_text(t)
  print(f, len(t.splitlines()))
