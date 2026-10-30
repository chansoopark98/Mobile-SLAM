// Original Author: Erik Murphy-Chutorian
//
// String alias for std::string, allowing reference runtime
// to more easily swap out string implementations in the future.

#pragma once

#include <string>

namespace c8 {

using String = std::string;

}  // namespace c8
