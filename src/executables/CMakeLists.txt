#ds export assembly as library
add_library(srrg_proslam_slam_assembly_library slam_assembly.cpp)
target_link_libraries(srrg_proslam_slam_assembly_library
  srrg_proslam_map_optimization_library
  srrg_proslam_motion_estimation_library
  srrg_proslam_relocalization_library
  srrg_proslam_visualization_library
  srrg_messages_library
  srrg_system_utils_library
)

#ds main executable
add_executable(srrg_proslam_app srrg_proslam_app.cpp)
target_link_libraries(srrg_proslam_app srrg_proslam_slam_assembly_library boost_system -pthread)

#ds ROS related - only build it if ROS is present (blindly link against the whole catkin library collection)
if(${SRRG_PROSLAM_HAS_ROS})
add_executable(srrg_proslam_node srrg_proslam_node.cpp)
target_link_libraries(srrg_proslam_node srrg_proslam_slam_assembly_library ${catkin_LIBRARIES})
endif()

#ds add kitti benchmark evaluation (snippet)
add_executable(kitti_evaluate_odometry kitti_evaluate_odometry.cpp)
