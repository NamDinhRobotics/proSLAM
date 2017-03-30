cmake_minimum_required(VERSION 2.8.3)
project(srrg_proslam)

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

#ds OpenCV - OpenCV_DIR might be overwritten by user
find_package(OpenCV REQUIRED)
message("using OpenCV version '${OpenCV_VERSION}' (${OpenCV_DIR})")

find_package(Eigen3 REQUIRED)
find_package(QGLViewer REQUIRED)

#ds check if qt5 is available - lock variable to false if qt4 should be used
set(SRRG_PROSLAM_USE_QT_5 false)
find_package(Qt5Widgets)
if(${Qt5Widgets_FOUND})

  #ds only enable qt5 if ros kinetic is present for now
  if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    set(SRRG_PROSLAM_USE_QT_5 true)
  endif()
endif()

#ds depending on found qt installation
if(${SRRG_PROSLAM_USE_QT_5})
  message("using Qt version '5'")
  find_package(Qt5 COMPONENTS Xml OpenGL Widgets REQUIRED)
else()
  message("using Qt version '4'")
  find_package(Qt4 REQUIRED)
endif()

find_package(SuiteSparse REQUIRED)

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
    g2o_catkin
    roscpp
    sensor_msgs
    cv_bridge
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    srrg_core
    srrg_gl_helpers
    srrg_core_viewers
    srrg_hbst
    g2o_catkin
  )
endif()

#ds flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -O0 -g -fPIC -fstack-check")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -Ofast -DNDEBUG -fPIC")

#ds specify additional locations of header files
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${g2o_catkin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  src
)

#ds specific includes: qt
if(${SRRG_PROSLAM_USE_QT_5})
  include_directories(${Qt5Xml_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
else()
  include_directories(${QT_INCLUDES})
endif()

#ds set catkin package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES srrg_proslam_types_library srrg_proslam_ui_library srrg_proslam_core_library
)

#ds set sources
add_subdirectory(src)
