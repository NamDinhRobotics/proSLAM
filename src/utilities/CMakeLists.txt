add_library(srrg_gslam_utilities_library
  gt_closure_buffer.cpp
  gt_closure_checker.cpp
)

target_link_libraries(srrg_gslam_utilities_library
  srrg_gslam_types_library
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARY}
)
