# Install script for directory: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/internal/ceres/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/autodiff_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/autodiff_first_order_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/autodiff_manifold.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/c_api.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/ceres.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/conditioned_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/constants.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/context.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/cost_function_to_functor.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/covariance.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/crs_matrix.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/cubic_interpolation.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/dynamic_autodiff_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/dynamic_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/dynamic_cost_function_to_functor.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/dynamic_numeric_diff_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/evaluation_callback.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/first_order_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/gradient_checker.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/gradient_problem.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/gradient_problem_solver.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/iteration_callback.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/jet.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/jet_fwd.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/line_manifold.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/loss_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/manifold.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/manifold_test_utils.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/normal_prior.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/numeric_diff_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/numeric_diff_first_order_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/numeric_diff_options.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/ordered_groups.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/problem.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/product_manifold.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/rotation.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/sized_cost_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/solver.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/sphere_manifold.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/tiny_solver.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/tiny_solver_autodiff_function.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/tiny_solver_cost_function_adapter.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/types.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/version.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/array_selector.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/autodiff.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/disable_warnings.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/eigen.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/euler_angles.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/fixed_array.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/householder_vector.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/integer_sequence_algorithm.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/jet_traits.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/line_parameterization.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/memory.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/numeric_diff.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/parameter_dims.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/port.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/reenable_warnings.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/sphere_manifold_functions.h"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/include/ceres/internal/variadic_evaluate.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal/miniglog/glog" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres-solver-src/internal/ceres/miniglog/glog/logging.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake"
         "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE RENAME "CeresConfig.cmake" FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/CeresConfig-install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/CeresConfigVersion.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/ceres_build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
