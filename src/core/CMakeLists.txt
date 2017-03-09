add_library(srrg_proslam_core_library
  mapper.cpp
  relocalizer.cpp
  tracker.cpp
)

target_link_libraries(srrg_proslam_core_library
  srrg_proslam_visualization_library
  ${g2o_catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
)
