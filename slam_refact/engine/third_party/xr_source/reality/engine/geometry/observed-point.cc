// Original Author: Dat Chu
//

#include "bzl/inliner/rules2.h"

cc_library {
  hdrs = {
    "observed-point.h",
  };
  deps = {
    "//c8:hpoint",
    "//c8/string:format",
    "//c8:string",
  };
  copts = {
    "-Wno-unused-private-field",
  };
}
cc_end(0x71c239c2);
