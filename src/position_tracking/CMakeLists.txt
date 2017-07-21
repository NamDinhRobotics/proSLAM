add_library(srrg_proslam_position_tracking_library
  base_tracker.cpp
  stereo_tracker.cpp
  depth_tracker.cpp
)

target_link_libraries(srrg_proslam_position_tracking_library
  srrg_proslam_aligners_library
  srrg_proslam_framepoint_generation_library
)
