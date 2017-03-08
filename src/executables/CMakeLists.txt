add_executable(srrg_proslam_app srrg_proslam_app.cpp)
#add_executable(kitti_evaluate_odometry kitti_evaluate_odometry.cpp)

target_link_libraries(srrg_proslam_app
  srrg_proslam_core_library
  ${catkin_LIBRARIES}
)
