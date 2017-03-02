add_library(srrg_proslam_ui_library
  gt_tracker_viewer.cpp
  gt_tracking_context_viewer.cpp
  simple_point_viewer.cpp
  aligner_viewer.cpp
)

target_link_libraries(srrg_proslam_ui_library
  srrg_proslam_types_library
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  srrg_gl_helpers_library
  srrg_core_viewers_library
  GL GLU
)

#ds package specific linkage: qt
if(${SRRG_proslam_USE_QT_5})
  target_link_libraries(srrg_proslam_ui_library
    Qt5::Widgets
  )
else()
  target_link_libraries(srrg_proslam_ui_library
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTXML_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
  )
endif()
