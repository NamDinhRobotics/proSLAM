add_library(srrg_gslam_core_library
  gt_mapper.cpp
  gt_relocalizer.cpp
  gt_tracker.cpp
  tracker_svi.cpp
)

target_link_libraries(srrg_gslam_core_library
  srrg_gslam_utilities_library
  srrg_gslam_ui_library
#  ${DLIB_LIBRARY}
#  ${DBOW2_LIBRARY}
)
