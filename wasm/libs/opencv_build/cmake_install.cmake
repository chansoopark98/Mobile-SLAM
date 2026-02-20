# Install script for directory: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/AlvaAR/src/libs/opencv

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv4/3rdparty" TYPE STATIC_LIBRARY OPTIONAL FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/3rdparty/lib/libade.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlicensesx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/licenses/opencv4" TYPE FILE RENAME "ade-LICENSE" FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/3rdparty/ade/ade-0.1.1f/LICENSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv4/opencv2" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/cvconfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv4/opencv2" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/opencv2/opencv_modules.hpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4/OpenCVModules.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4/OpenCVModules.cmake"
         "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/CMakeFiles/Export/lib/cmake/opencv4/OpenCVModules.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4/OpenCVModules-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4/OpenCVModules.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/CMakeFiles/Export/lib/cmake/opencv4/OpenCVModules.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4" TYPE FILE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/CMakeFiles/Export/lib/cmake/opencv4/OpenCVModules-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4" TYPE FILE FILES
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/unix-install/OpenCVConfig-version.cmake"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/unix-install/OpenCVConfig.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xscriptsx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/CMakeFiles/install/setup_vars_opencv4.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv4" TYPE FILE FILES
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/AlvaAR/src/libs/opencv/platforms/scripts/valgrind.supp"
    "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/AlvaAR/src/libs/opencv/platforms/scripts/valgrind_3rdparty.supp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/3rdparty/zlib/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/include/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/calib3d/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/core/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/dnn/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/features2d/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/flann/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/gapi/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/highgui/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/imgcodecs/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/imgproc/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/java/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/js/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/ml/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/objc/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/objdetect/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/photo/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/python/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/stitching/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/ts/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/video/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/videoio/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/.firstpass/world/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/core/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/flann/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/imgproc/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/python_tests/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/features2d/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/calib3d/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/video/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/java_bindings_generator/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/js_bindings_generator/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/objc_bindings_generator/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/modules/python_bindings_generator/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/doc/cmake_install.cmake")
  include("/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/data/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/wasm/libs/opencv_build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
