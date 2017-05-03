add_library(srrg_proslam_types_library
  frame.cpp
  local_map.cpp
  world_map.cpp
  frame_point.cpp
  landmark.cpp
  camera.cpp
  parameters.cpp
)

target_link_libraries(srrg_proslam_types_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)
