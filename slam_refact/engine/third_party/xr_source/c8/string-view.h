// Original Author: Erik Murphy-Chutorian
//
// StringView alias for std::string_view, allowing reference runtime
// to use more consistent class naming.

#pragma once

#include <string_view>

namespace c8 {

using StringView = std::string_view;

}  // namespace c8
