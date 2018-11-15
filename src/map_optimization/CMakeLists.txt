add_library(srrg_proslam_map_optimization_library
  graph_optimizer.cpp
)

target_link_libraries(srrg_proslam_map_optimization_library
  srrg_proslam_types_library
  ${g2o_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARY}
)
