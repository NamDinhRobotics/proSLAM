add_library(srrg_proslam_visualization_library
  viewer_input_images.cpp
  viewer_output_map.cpp
)

target_link_libraries(srrg_proslam_visualization_library
  srrg_proslam_types_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  srrg_gl_helpers_library
  srrg_core_viewers_library
  GL GLU
)

#ds package specific linkage: qt
if(${SRRG_PROSLAM_USE_QT_5})
  target_link_libraries(srrg_proslam_visualization_library
    Qt5::Widgets
  )
else()
  target_link_libraries(srrg_proslam_visualization_library
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTXML_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
  )
endif()
