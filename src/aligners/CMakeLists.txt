add_library(srrg_proslam_aligners_library
  stereouv_aligner.cpp
  xyz_aligner.cpp
  correspondence_collection.cpp
)

target_link_libraries(srrg_proslam_aligners_library
  srrg_proslam_types_library
)
