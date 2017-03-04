add_library(srrg_proslam_core_library
  mapper.cpp
  relocalizer.cpp
  tracker.cpp
)

target_link_libraries(srrg_proslam_core_library
  srrg_proslam_visualization_library
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARY}
)
