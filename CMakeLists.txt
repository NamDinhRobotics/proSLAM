cmake_minimum_required(VERSION 2.8.3)
project(srrg_proslam)

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})

#ds determine build type, default build type: release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE)
endif()
message("${PROJECT_NAME}|build type: '${CMAKE_BUILD_TYPE}'")

#ds flags for release build
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RELEASE")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -Wall -pedantic -Ofast -DNDEBUG")
  message("${PROJECT_NAME}|adding flags for '${CMAKE_BUILD_TYPE}': '-std=c++11 -march=native -Wall -pedantic -Ofast -DNDEBUG'")
  
#ds flags for other build(s) (e.g. debug)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -Wall -pedantic -O0 -g -fstack-check")
  message("${PROJECT_NAME}|adding flags for '${CMAKE_BUILD_TYPE}': '-std=c++11 -march=native -Wall -pedantic -O0 -g -fstack-check'")
endif()

#ds enable ARM flags if applicable
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("${PROJECT_NAME}|enabling ARM neon optimizations")
endif()

#ds specify target binary descriptor bit size (256 if not defined)
add_definitions(-DSRRG_PROSLAM_DESCRIPTOR_SIZE_BITS=256)

#ds specify target log level: 0 ERROR, 1 WARNING, 2 INFO, 3 DEBUG (defaults to 2 if not defined)
add_definitions(-DSRRG_PROSLAM_LOG_LEVEL=2)

#ds enable descriptor merging in HBST (and other SRRG components) - careful for collisions with landmark merging!
add_definitions(-DSRRG_MERGE_DESCRIPTORS)

#ds load Eigen library
find_package(Eigen3 REQUIRED)
message("${PROJECT_NAME}|using Eigen version: '3' (${EIGEN3_INCLUDE_DIR})")

#ds enable Eigen for HBST
add_definitions(-DSRRG_HBST_HAS_EIGEN)

#ds check if a supported ros version is installed to determine which packages we include
set(SRRG_PROSLAM_HAS_ROS false)
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "indigo" OR "$ENV{ROS_DISTRO}" STREQUAL "melodic")

  #ds ROS support enabled
  message("${PROJECT_NAME}|using ROS version: '$ENV{ROS_DISTRO}' (building nodes)")
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
    image_geometry
  )
  
  #ds switch to ROS opencv libraries without calling find_package to avoid conflicts with other existing OpenCV installations
  set(OpenCV_DIR "ROS")
  set(OpenCV_LIBS ${catkin_LIBRARIES})
  message("${PROJECT_NAME}|using ROS OpenCV version: '${OpenCV_VERSION}' (${OpenCV_DIR})")
  add_definitions(-DSRRG_HBST_HAS_OPENCV)
else()

  #ds build proslam without ROS components
  find_package(catkin REQUIRED COMPONENTS
    srrg_core
    srrg_gl_helpers
    srrg_core_viewers
    srrg_hbst
  )
  
  #ds OpenCV - OpenCV_DIR might be overwritten by user
  find_package(OpenCV REQUIRED)
  message("${PROJECT_NAME}|using OpenCV version: '${OpenCV_VERSION}' (${OpenCV_DIR})")
  add_definitions(-DSRRG_HBST_HAS_OPENCV)
endif()

#ds check for OpenCV contrib library (optional)
string(FIND "${OpenCV_LIBS}" "xfeatures2d" FOUND_OPENCV_CONTRIB)
if(NOT ${FOUND_OPENCV_CONTRIB} EQUAL -1)
  message("${PROJECT_NAME}|found xfeatures2d library, building contributed OpenCV components")
  add_definitions(-DSRRG_PROSLAM_HAS_OPENCV_CONTRIB)
else()
  message("${PROJECT_NAME}|xfeatures2d library not found, using ORB instead of BRIEF descriptors")
endif()

#ds load QGLViewer library
find_package(QGLViewer REQUIRED)

#ds attempt to locate a g2o package through cmake modules
find_package(G2O QUIET)

#ds set ownership model for g2o TODO retrieve this information from g2o (current g2o requires cmake 3+)
if (${CMAKE_MAJOR_VERSION} GREATER 2)

  #ds (comment this line if you're using an older g2o version but cmake 3+)
  add_definitions(-DSRRG_PROSLAM_G2O_HAS_NEW_OWNERSHIP_MODEL)
  message("${PROJECT_NAME}|found CMake 3+: assuming new g2o ownership model")
else()

  #ds old g2o ownership model is used
  message("${PROJECT_NAME}|found CMake 2: assuming old g2o ownership model")
endif()

#ds check if a custom g2o library is installed
set(SRRG_PROSLAM_HAS_OWN_G2O false)

#ds if theres no SRRG g2o installation
if("${G2O_SRRG_DIR}" STREQUAL "")

  #ds check if theres also no srrg g2o installation
  if("$ENV{G2O_ROOT}" STREQUAL "")
  
    #ds no custom g2o installation found, fallback to catkin g2o
    message("${PROJECT_NAME}|using catkin g2o")
  else()
  
    #ds use custom g2o
    message("${PROJECT_NAME}|using custom g2o: '$ENV{G2O_ROOT}'")
    set(SRRG_PROSLAM_HAS_OWN_G2O true)
  endif()
else()

  #ds use srrg g2o
  message("${PROJECT_NAME}|using SRRG g2o: '${G2O_SRRG_DIR}'")
  set(SRRG_PROSLAM_HAS_OWN_G2O true)
endif()

#ds if a custom g2o package was found
if(SRRG_PROSLAM_HAS_OWN_G2O)

  #ds add it to our variables
  set(g2o_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
  set(g2o_LIBRARIES ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

else()

  #ds attempt to find a catkin g2o
  find_package(g2o_catkin REQUIRED)
  
    #ds add it to our variables
  set(g2o_INCLUDE_DIRS ${g2o_catkin_INCLUDE_DIRS})
  set(g2o_LIBRARIES ${g2o_catkin_LIBRARIES})
endif()

#ds load suite sparse for pose graph optimization
find_package(SuiteSparse REQUIRED)

#ds specify additional locations of header files
#ds treating them as system includes to surpress warnings (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${g2o_INCLUDE_DIRS}
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
  ${g2o_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${SRRG_QT_INCLUDE_DIRS}
  src

  LIBRARIES
  
  #ds package libraries
  srrg_proslam_aligners_library 
  srrg_proslam_framepoint_generation_library 
  srrg_proslam_map_optimization_library 
  srrg_proslam_position_tracking_library 
  srrg_proslam_relocalization_library 
  srrg_proslam_types_library 
  srrg_proslam_visualization_library 
  srrg_proslam_slam_assembly_library
  
  #ds export the used libraries in this project (consistency for integration)
  ${g2o_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${OpenCV_LIBS}
  yaml-cpp
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY}
  ${SRRG_QT_LIBRARIES}
)

#ds set sources
add_subdirectory(src)
add_subdirectory(executables)
