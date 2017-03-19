add_library(srrg_proslam_map_management_library
  graph_optimizer.cpp
)

target_link_libraries(srrg_proslam_map_management_library
  srrg_proslam_types_library
  ${g2o_catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
)
