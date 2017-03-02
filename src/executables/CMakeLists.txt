add_executable(srrg_gslam_mapper_app srrg_gslam_mapper_app.cpp)
add_executable(srrg_gslam_kitti_app srrg_gslam_kitti_app.cpp)
add_executable(srrg_gslam_vibike_app srrg_gslam_vibike_app.cpp)
add_executable(srrg_gslam_euroc_app srrg_gslam_euroc_app.cpp)
add_executable(srrg_kitti_evaluate_odometry srrg_kitti_evaluate_odometry.cpp)

target_link_libraries(srrg_gslam_mapper_app
  srrg_gslam_core_library
  srrg_txt_io_library
  srrg_system_utils_library
  ${OpenCV_LIBS}
)

target_link_libraries(srrg_gslam_kitti_app
  srrg_gslam_core_library
  srrg_txt_io_library
  srrg_system_utils_library
  ${OpenCV_LIBS}
)

target_link_libraries(srrg_gslam_vibike_app
  srrg_gslam_core_library
  srrg_txt_io_library
  srrg_system_utils_library
  ${OpenCV_LIBS}
)

target_link_libraries(srrg_gslam_euroc_app
  srrg_gslam_core_library
  srrg_txt_io_library
  srrg_system_utils_library
  ${OpenCV_LIBS}
)

target_link_libraries(srrg_kitti_evaluate_odometry
  srrg_gslam_types_library
  ${OpenCV_LIBS}
)
