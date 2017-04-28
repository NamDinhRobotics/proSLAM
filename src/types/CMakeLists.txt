add_library(srrg_proslam_types_library
  frame.cpp
  local_map.cpp
  world_map.cpp
  frame_point.cpp
  landmark.cpp
  camera.cpp
  parameter.cpp
)

target_link_libraries(srrg_proslam_types_library
  ${OpenCV_LIBS}
  yaml-cpp
)
