add_executable(srrg_proslam_app srrg_proslam_app.cpp)
target_link_libraries(srrg_proslam_app
  srrg_proslam_map_optimization_library
  srrg_proslam_motion_estimation_library
  srrg_proslam_relocalization_library
  srrg_proslam_visualization_library
)

#ds ROS related - only build it if ROS is present
if(${SRRG_PROSLAM_HAS_ROS})
add_executable(srrg_proslam_node srrg_proslam_node.cpp)
target_link_libraries(srrg_proslam_node
  srrg_proslam_map_optimization_library
  srrg_proslam_motion_estimation_library
  srrg_proslam_relocalization_library
  srrg_proslam_visualization_library
)
endif()

add_executable(kitti_evaluate_odometry kitti_evaluate_odometry.cpp)
target_link_libraries(kitti_evaluate_odometry ${OpenCV_LIBS})
