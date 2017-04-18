add_library(srrg_proslam_aligners_library
  base_frame_aligner.cpp
  stereouv_aligner.cpp
  uvd_aligner.cpp
  base_local_map_aligner.cpp
  xyz_aligner.cpp
)

target_link_libraries(srrg_proslam_aligners_library
  srrg_proslam_types_library
)
