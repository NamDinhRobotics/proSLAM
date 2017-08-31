add_library(srrg_proslam_types_library
  parameters.cpp
  frame.cpp
  local_map.cpp
  world_map.cpp
  frame_point.cpp
  landmark.cpp
  camera.cpp
)

target_link_libraries(srrg_proslam_types_library
  srrg_system_utils_library
  ${OpenCV_LIBS}
  yaml-cpp
)
