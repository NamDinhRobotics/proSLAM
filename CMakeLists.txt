cmake_minimum_required(VERSION 2.8.3)
project(srrg_gslam)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

#ds DBoW2 configuration
#set(DLIB_INCLUDE_DIR "$ENV{HOME}/source/libraries/DLib/include")
#set(DLIB_LIBRARY "$ENV{HOME}/source/libraries/DLib/build/release/libDLib.so")
#set(DBOW2_INCLUDE_DIR "$ENV{HOME}/source/libraries/DBoW2/include")
#set(DBOW2_LIBRARY "$ENV{HOME}/source/libraries/DBoW2/build/release/libDBoW2.so")

#ds OpenCV: we do not want to use the kinect opencv(3) package
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")

  #ds use our custom opencv
  set(OpenCV_DIR "$ENV{HOME}/source/libraries/opencv/build/release")
  find_package(OpenCV REQUIRED)
  message("using custom OpenCV ${OpenCV_VERSION} (${OpenCV_DIR})")

else()

  #ds regular find (TODO enforce custom opencv installation?)
  find_package(OpenCV REQUIRED)
  message("using system OpenCV ${OpenCV_VERSION} (${OpenCV_DIR})")

endif()

find_package(Eigen3 REQUIRED)
find_package(QGLViewer REQUIRED)

#ds check if qt5 is available - lock variable to false if qt4 should be used
set(SRRG_GSLAM_USE_QT_5 false)
find_package(Qt5Widgets)
if(${Qt5Widgets_FOUND})

  #ds only enable qt5 if ros kinetic is present
  if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    set(SRRG_GSLAM_USE_QT_5 true)
  endif()
endif()

#ds depending on found qt installation
if(${SRRG_GSLAM_USE_QT_5})
  message("using Qt 5")
  find_package(Qt5 COMPONENTS Xml OpenGL Widgets REQUIRED)
else()
  message("using Qt 4")
  find_package(Qt4 REQUIRED)
endif()

find_package(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)

#ds find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  srrg_core
  srrg_gl_helpers
  srrg_core_viewers
  srrg_hbst
  srrg_core_map
  roscpp
  sensor_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

###########
## Build ##
###########

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -O0 -g -fPIC -fstack-check")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -O3 -DNDEBUG -fPIC")

## Specify additional locations of header files
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
#  ${DLIB_INCLUDE_DIR}
#  ${DBOW2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  src
)

#ds specific includes: qt
if(${SRRG_GSLAM_USE_QT_5})
  include_directories(${Qt5Xml_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
else()
  include_directories(${QT_INCLUDES})
endif()

#ds set sources
add_subdirectory(src)

#############
## Install ##
#############

#############
## Testing ##
#############
