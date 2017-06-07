cmake_minimum_required(VERSION 2.8.3)
project(srrg_proslam)

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})

#ds determine build type, default build type Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("build type: '${CMAKE_BUILD_TYPE}'")

#ds specify logging level, 0(or not defined): DEBUG, 1: INFO, 2: WARNING, 3: ERROR)
add_definitions(-DSRRG_PROSLAM_LOG_LEVEL=0)

#ds specify target binary descriptor bit size (256 if not defined)
add_definitions(-DSRRG_PROSLAM_DESCRIPTOR_SIZE_BITS=256)

#ds specify target binary descriptor maximum acceptable matching distance (256 if not defined)
add_definitions(-DSRRG_PROSLAM_DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE=25)

#ds OpenCV - OpenCV_DIR might be overwritten by user
find_package(OpenCV REQUIRED)
message("using OpenCV version: '${OpenCV_VERSION}' (${OpenCV_DIR})")

#ds enable OpenCV for HBST
add_definitions(-DSRRG_HBST_HAS_OPENCV)

#ds load Eigen library
find_package(Eigen3 REQUIRED)
message("using Eigen version: '3' (${EIGEN3_INCLUDE_DIR})")

#ds enable Eigen for HBST
add_definitions(-DSRRG_HBST_HAS_EIGEN)

#ds load QGLViewer library - inherently also loads a matching Qt version
find_package(QGLViewer REQUIRED)

#ds check if a supported ros version is installed to determine which packages we include
set(SRRG_PROSLAM_HAS_ROS false)
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "indigo")

  #ds ROS support enabled
  message("using ROS version: '$ENV{ROS_DISTRO}' (building nodes)")
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

  #ds build proslam without ROS components
  find_package(catkin REQUIRED COMPONENTS
    srrg_core
    srrg_gl_helpers
    srrg_core_viewers
    srrg_hbst
  )
endif()

#ds attempt to find g2o_slim in catkin stack
find_package(g2o_slim_edu QUIET)
if(g2o_slim_edu_FOUND)

  #ds configure cmake for custom g2o usage
  message("using g2o_slim_edu: '${g2o_slim_edu_INCLUDE_DIRS}'")
  add_definitions(-DSRRG_PROSLAM_USE_G3O)
  set(g2o_catkin_INCLUDE_DIRS ${g2o_slim_edu_INCLUDE_DIRS})
  set(g2o_catkin_LIBRARIES ${g2o_slim_edu_LIBRARIES})
  
#ds look for an official g2o version
else()

  #ds attempt to locate a g2o package
  find_package(G2O QUIET)
    
  #ds check if a custom g2o library is installed
  set(SRRG_PROSLAM_HAS_OWN_G2O false)
    
  #ds if theres no SRRG g2o installation
  if("${G2O_SRRG_DIR}" STREQUAL "")
    
    #ds check if theres also no srrg g2o installation
    if("$ENV{G2O_ROOT}" STREQUAL "")
      
      #ds no custom g2o installation found, fallback to catkin g2o
      message("using catkin g2o")
    else()
      
      #ds use custom g2o
      message("using custom g2o: '$ENV{G2O_ROOT}'")
      set(SRRG_PROSLAM_HAS_OWN_G2O true)
    endif()
  else()
    
    #ds use srrg g2o
    message("using SRRG g2o: '${G2O_SRRG_DIR}'")
    set(SRRG_PROSLAM_HAS_OWN_G2O true)
  endif()
    
  #ds if a custom g2o package was found
  if(SRRG_PROSLAM_HAS_OWN_G2O)
      
    #ds add it to the catkin variables - this way we can support also g2o catkin without changing variable names
    set(g2o_catkin_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
      
    #ds link against the libraries we require
    set(g2o_catkin_LIBRARIES ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY})
      
  else()
    find_package(g2o_catkin REQUIRED)
  endif()
endif()

#ds load suite sparse for pose graph optimization
find_package(SuiteSparse REQUIRED)

#ds check flags for release build
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RELEASE")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -Ofast -DNDEBUG -fPIC")
  message("adding flags for '${CMAKE_BUILD_TYPE}': '--std=c++11 -Wall -Ofast -DNDEBUG -fPIC'")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall -O0 -g -fPIC -fstack-check")
  message("adding flags for '${CMAKE_BUILD_TYPE}': '--std=c++11 -Wall -O0 -g -fPIC -fstack-check'")
endif()

#ds enable ARM flags if applicable
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
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

#ds help the catkin tool on 16.04 (cmake seems unable to find single libraries, although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

#ds set up catkin package (exported components)
catkin_package(
  INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIR}
  ${g2o_catkin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${SRRG_QT_INCLUDE_DIRS}
  src

  LIBRARIES
  srrg_proslam_aligners_library 
  srrg_proslam_framepoint_generation_library 
  srrg_proslam_map_optimization_library 
  srrg_proslam_motion_estimation_library 
  srrg_proslam_relocalization_library 
  srrg_proslam_types_library 
  srrg_proslam_visualization_library 
  srrg_proslam_slam_assembly_library
)

#ds set sources
add_subdirectory(src)
