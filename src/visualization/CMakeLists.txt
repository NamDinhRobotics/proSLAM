add_library(srrg_proslam_visualization_library
  viewer_input_images.cpp
  viewer_output_map.cpp
)

target_link_libraries(srrg_proslam_visualization_library
  srrg_proslam_types_library
  srrg_core_viewers_library
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY}
  ${SRRG_QT_LIBRARIES}
)
