#pragma once
#include "srrg_core_viewers/simple_viewer.h"
#include "types/gt_defs.h"

namespace gslam {

  class AlignerViewer: public srrg_core_viewers::SimpleViewer{

  public:

    AlignerViewer();

    void setTransformQueryToReferenceGuess(const TransformMatrix3D& transform_query_to_reference_);
    void setTransformQueryToReference(const TransformMatrix3D& transform_query_to_reference_);
    void setPointsQuery(const std::vector<PointDrawable>& points_) {_points_query = points_;}
    void setPointsReference(const std::vector<PointDrawable>& points_) {_points_reference = points_;}

  protected:
    
    virtual void draw();

    virtual void keyPressEvent(QKeyEvent* event_);

    virtual QString helpString( ) const;

  protected:

    TransformMatrix3D _transform_query_to_reference = TransformMatrix3D::Identity();
    std::vector<PointDrawable> _points_query;
    std::vector<PointDrawable> _points_query_guess;
    std::vector<PointDrawable> _points_reference;
    std::vector<PointDrawable> _points_query_transformed;
    bool _apply_transform = false;
    bool _draw_verbose    = false;

  };
}
