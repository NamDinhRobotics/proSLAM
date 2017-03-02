#pragma once
#include "srrg_core_viewers/simple_viewer.h"
#include "types/gt_defs.h"

namespace gslam {

  class SimplePointViewer: public srrg_core_viewers::SimpleViewer{

  public:

    SimplePointViewer();

    void setPoints(const std::vector<PointDrawable>& points_) {_points = points_;}

  protected:
    
    virtual void draw();

    virtual void keyPressEvent(QKeyEvent* event_);

    virtual QString helpString( ) const;

  protected:

    std::vector<PointDrawable> _points;

  };
}
