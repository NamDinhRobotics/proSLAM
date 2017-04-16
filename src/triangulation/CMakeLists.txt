add_library(srrg_proslam_triangulation_library
  base_framepoint_generator.cpp
  stereo_triangulator.cpp
)

target_link_libraries(srrg_proslam_triangulation_library
  srrg_proslam_types_library
)
