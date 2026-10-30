// Original Author: Erik Murphy-Chutorian
//
// Vector alias for std::vector, allowing reference runtime
// to more easily swap out vector implementations in the future.

#pragma once

#include <type_traits>
#include <vector>

namespace c8 {

namespace _ {

// Use a typedef class so we can remove the std::vector<bool> partial
// specification.
template <typename T>
struct Vector {
  static_assert(
    !std::is_same<std::remove_cv_t<T>, bool>(),
    "Vector<bool> removed since underlying std::vector<bool> is not a real container. It doesn't "
    "return bool& and its bit operations might not be fast.");
  typedef std::vector<T> type;
};

}  // namespace _

// Use alias declaration for creating the actual type.
template <typename T>
using Vector = typename _::Vector<T>::type;

}  // namespace c8
