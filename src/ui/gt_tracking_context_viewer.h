#pragma once
#include "core/gt_relocalizer.h"
#include "srrg_core_viewers/simple_viewer.h"
#include "types/contexts/gt_world_context.h"

namespace gslam {

  class TrackingContextViewer: public srrg_core_viewers::SimpleViewer{
  public:
    TrackingContextViewer(WorldContext* context_ = 0, const gt_real& object_scale_ = 0.1);

    inline bool landmarksDrawn() const {return _landmarks_drawn;}
    inline void setLandmarksDrawn(bool landmarks_drawn_) {_landmarks_drawn=landmarks_drawn_;}
    
    inline bool framesDrawn() const {return _frames_drawn;}
    inline void setFramesDrawn(bool frames_drawn_) {_frames_drawn=frames_drawn_;}

    inline bool followRobot() const {return _follow_robot;}
    inline void setFollowRobot(bool follow_robot_) {_follow_robot=follow_robot_;}

    void setViewpointOrigin(const TransformMatrix3D& world_to_robot_origin_) {_world_to_robot_origin = world_to_robot_origin_;}

  protected:
    
    virtual void draw();

    virtual void keyPressEvent(QKeyEvent* event_);

    virtual QString helpString( ) const;

    void drawFrame(const Frame* frame_, const Vector3& color_rgb_);
    void drawLandmarks();
    void drawOdometry();

  protected:

    WorldContext* _context   = 0;
    bool _frames_drawn       = false;
    bool _key_frames_drawn   = true;
    bool _landmarks_drawn    = true;
    bool _follow_robot       = true;
    bool _ground_truth_drawn = false;
    gt_real _object_scale    = 0.1;
    gt_real _point_size      = 1;
    TransformMatrix3D _world_to_robot_origin = TransformMatrix3D::Identity();
  };

}
