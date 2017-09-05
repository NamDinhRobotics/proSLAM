#ds main executable (pthread is used for the GUI thread)
add_executable(srrg_proslam_app app.cpp)
target_link_libraries(srrg_proslam_app srrg_proslam_slam_assembly_library -pthread boost_system)
add_executable(app app.cpp)
target_link_libraries(app srrg_proslam_slam_assembly_library -pthread boost_system)

#ds ROS related - only build it if ROS is present (blindly link against the whole catkin library collection)
if(${SRRG_PROSLAM_HAS_ROS})
  add_executable(srrg_proslam_node srrg_proslam_node.cpp)
  target_link_libraries(srrg_proslam_node srrg_proslam_slam_assembly_library ${catkin_LIBRARIES})
endif()

#ds add kitti benchmark evaluation (snippet)
add_executable(kitti_evaluate_odometry kitti_evaluate_odometry.cpp)

#ds g2o to kitti trajectory converter (offline bundle adjustment testing)
add_executable(trajectory_converter trajectory_converter.cpp)
