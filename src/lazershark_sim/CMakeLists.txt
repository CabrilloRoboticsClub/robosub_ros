cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(lazershark_sim)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
find_package(ament_cmake REQUIRED)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

# --------------------------------------------------------------------------- #
# Compile as C++14.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

# Harmonic (default)
# if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
#   find_package(gz-cmake3 REQUIRED)
#   set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#   gz_find_package(gz-common5 REQUIRED)
#   set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#   gz_find_package(gz-rendering8 REQUIRED)
#   set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

#   gz_find_package(gz-sim8 REQUIRED)
#   set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

#   message(STATUS "Compiling against Gazebo Harmonic")
# # Garden
# elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
#   find_package(gz-cmake3 REQUIRED)
#   set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#   gz_find_package(gz-common5 REQUIRED)
#   set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#   gz_find_package(gz-rendering7 REQUIRED)
#   set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
  
#   gz_find_package(gz-sim7 REQUIRED)
#   set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

#   message(STATUS "Compiling against Gazebo Garden")
# else()  
#   message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
# endif()

# # --------------------------------------------------------------------------- #
# find_package(RapidJSON REQUIRED)
# find_package(OpenCV REQUIRED)

# pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)


install(DIRECTORY config robot worlds launch
  DESTINATION share/${PROJECT_NAME}
)

if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

  ament_package()
endif()
