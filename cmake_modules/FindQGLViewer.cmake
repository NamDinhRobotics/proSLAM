#ds check if qt5 is available - otherwise fallback to qt4
find_package(Qt5Xml)

# Need to find both Qt4 and QGLViewer if the QQL support is to be built
if(${Qt5Xml_FOUND})
  find_package(Qt5 COMPONENTS Xml OpenGL Gui)
else()
  find_package(Qt4 COMPONENTS QtXml QtOpenGL QtGui)
endif()

find_path(QGLVIEWER_INCLUDE_DIR qglviewer.h
  /usr/include/QGLViewer
  /opt/local/include/QGLViewer
  /usr/local/include/QGLViewer
  /sw/include/QGLViewer
)

if(${Qt5Xml_FOUND})
  find_library(QGLVIEWER_LIBRARY NAMES  qglviewer-qt5 QGLViewer
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
)
else()
  find_library(QGLVIEWER_LIBRARY NAMES  qglviewer-qt4 QGLViewer
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
)
endif()

if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND TRUE)
else(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND FALSE)
endif(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
