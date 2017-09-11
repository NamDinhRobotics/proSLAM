#ds main executable (pthread is used for the GUI thread)
add_executable(app app.cpp)
target_link_libraries(app srrg_proslam_slam_assembly_library -pthread boost_system)
add_executable(srrg_proslam_app app.cpp)
target_link_libraries(srrg_proslam_app srrg_proslam_slam_assembly_library -pthread boost_system)

#ds ROS related - only build it if ROS is present (blindly link against the whole catkin library collection)
if(${SRRG_PROSLAM_HAS_ROS})
  add_executable(node node.cpp)
  target_link_libraries(node srrg_proslam_slam_assembly_library ${catkin_LIBRARIES})
endif()

#ds add kitti benchmark evaluation (snippet)
add_executable(kitti_evaluate_odometry kitti_evaluate_odometry.cpp)

#ds g2o to kitti trajectory converter (offline bundle adjustment testing)
add_executable(trajectory_converter trajectory_converter.cpp)

#ds minimal stereo calibration suite
add_executable(stereo_calibrator stereo_calibrator.cpp)
target_link_libraries(stereo_calibrator ${OpenCV_LIBS})

#ds euroc trajectory analyzer (e.g. RMSE computation)
add_executable(trajectory_analyzer trajectory_analyzer.cpp)
target_link_libraries(trajectory_analyzer ${OpenCV_LIBS} srrg_core_types_library)

#ds g2o testing
add_executable(test_g2o_compatibility test_g2o_compatibility.cpp)
target_link_libraries(test_g2o_compatibility srrg_proslam_map_optimization_library)
