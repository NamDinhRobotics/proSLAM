add_library(srrg_proslam_types_library
  aligners/aligner_factory.cpp
  aligners/xyz_aligner.cpp
  aligners/stereouv_aligner.cpp
  contexts/frame.cpp
  contexts/local_map.cpp
  contexts/world_map.cpp
  contexts/correspondence_collection.cpp
  items/frame_point.cpp
  items/landmark.cpp
  camera.cpp
  stereo_grid_detector.cpp
)

target_link_libraries(srrg_proslam_types_library
  srrg_core_map_library
  ${OpenCV_LIBS}
)
