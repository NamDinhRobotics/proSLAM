add_library(srrg_proslam_motion_estimation_library
  base_tracker.cpp
  stereo_tracker.cpp
  depth_tracker.cpp
)

target_link_libraries(srrg_proslam_motion_estimation_library
  srrg_proslam_aligners_library
  srrg_proslam_frame_generation_library
)
