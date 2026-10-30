/* slam_refact excerpt of musl 1.2.5 src/include/features.h (emsdk 5.0.0,
 * system/lib/libc/musl/src/include/features.h): the vendored *_data.h headers include <features.h>
 * for the `hidden` macro. The public features.h is the host C library's (#include_next); kept from
 * musl: the `hidden` definition (verbatim). Dropped: weak, weak_alias (unused). musl is MIT licensed,
 * see ../COPYRIGHT. */
#ifndef FEATURES_H
#define FEATURES_H

#include_next <features.h>

#define hidden __attribute__((__visibility__("hidden")))

#endif
