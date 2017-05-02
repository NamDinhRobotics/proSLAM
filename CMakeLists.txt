cmake_minimum_required(VERSION 2.8.3)
project(srrg_proslam)

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
message("build type: ${CMAKE_BUILD_TYPE}")

#ds OpenCV - OpenCV_DIR might be overwritten by user
find_package(OpenCV REQUIRED)
message("using OpenCV version '${OpenCV_VERSION}' (${OpenCV_DIR})")

#ds enable OpenCV for HBST
add_definitions(-DSRRG_HBST_HAS_OPENCV)

#ds load eigen library
find_package(Eigen3 REQUIRED)

#ds load QGLViewer library - inherently also loads a matching Qt version
find_package(QGLViewer REQUIRED)

#ds check if a supported ros version is installed to determine which packages we include
set(SRRG_PROSLAM_HAS_ROS false)
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "indigo")
  message("using ROS version '$ENV{ROS_DISTRO}' (building nodes)")
  set(SRRG_PROSLAM_HAS_ROS true)
  find_package(catkin REQUIRED COMPONENTS
    srrg_core
    srrg_gl_helpers
    srrg_core_viewers
    srrg_hbst
    roscpp
    sensor_msgs
    cv_bridge
    nav_msgs
    message_filters
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    srrg_core
    srrg_gl_helpers
    srrg_core_viewers
    srrg_hbst
  )
endif()

#ds check if a custom g2o library is installed
set(SRRG_PROSLAM_HAS_OWN_G2O false)
if("$ENV{G2O_ROOT}" STREQUAL "")
  message("using catkin g2o")
else()
  message("using custom g2o: $ENV{G2O_ROOT}")
  set(SRRG_PROSLAM_HAS_OWN_G2O true)
endif()

#ds if a custom g2o was found
if(SRRG_PROSLAM_HAS_OWN_G2O)

  #ds locate g2o package
  find_package(G2O REQUIRED)
  
  #ds add it to the catkin variables
  set(g2o_catkin_INCLUDE_DIRS "${G2O_INCLUDE_DIR}")
  set(g2o_catkin_LIBRARIES ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY})
  
else()
  find_package(g2o_catkin REQUIRED)
endif()

#ds load suite sparse for pose graph optimization
find_package(SuiteSparse REQUIRED)

#ds flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -O0 -g -fPIC -fstack-check")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -Ofast -DNDEBUG -fPIC")

#ds enable ARM flags if applicable
if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

#ds specify additional locations of header files
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${g2o_catkin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${SRRG_QT_INCLUDE_DIRS}
  src
)

#ds set up catkin package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES srrg_proslam_types_library srrg_proslam_ui_library srrg_proslam_core_library
)

#ds set sources
add_subdirectory(src)
