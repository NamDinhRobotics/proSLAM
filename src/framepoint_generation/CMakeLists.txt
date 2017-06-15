add_library(srrg_proslam_framepoint_generation_library
  base_framepoint_generator.cpp
  stereo_framepoint_generator.cpp
  depth_framepoint_generator.cpp
)

target_link_libraries(srrg_proslam_framepoint_generation_library
  srrg_proslam_types_library
  srrg_nnkd_keypoint_detector_library
  ${Caffe_LIBRARIES}
)
