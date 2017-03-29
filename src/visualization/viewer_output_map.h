#pragma once
#include "srrg_core_viewers/simple_viewer.h"
#include "types/world_map.h"

namespace proslam {

  class ViewerOutputMap: public srrg_core_viewers::SimpleViewer{
  public:
    ViewerOutputMap(WorldMap* context_ = 0, const real& object_scale_ = 0.1, const std::string& window_name_ = "output: map");

    inline bool landmarksDrawn() const {return _landmarks_drawn;}
    inline void setLandmarksDrawn(const bool& landmarks_drawn_) {_landmarks_drawn = landmarks_drawn_;}
    
    inline bool framesDrawn() const {return _frames_drawn;}
    inline void setFramesDrawn(const bool& frames_drawn_) {_frames_drawn = frames_drawn_;}

    inline bool followRobot() const {return _follow_robot;}
    inline void setFollowRobot(const bool& follow_robot_) {_follow_robot = follow_robot_;}

    void setWorldToRobotOrigin(const TransformMatrix3D& world_to_robot_origin_) {_world_to_robot_origin = world_to_robot_origin_;}
    void setRotationRobotView(const TransformMatrix3D& rotation_robot_view_) {_rotation_robot_view = rotation_robot_view_;}
    void setIsOpen(const bool& is_open_) {_is_open = is_open_;}

  protected:
    
    virtual void draw();

    virtual void keyPressEvent(QKeyEvent* event_);

    virtual QString helpString( ) const;

    void drawFrame(const Frame* frame_, const Vector3& color_rgb_);
    void drawLandmarks();
    void drawLandmarksActiveOnly();

  protected:

    WorldMap* _context   = 0;
    bool _frames_drawn       = true;
    bool _local_maps_drawn   = true;
    bool _landmarks_drawn    = true;
    bool _follow_robot       = true;
    bool _ground_truth_drawn = false;
    bool _is_open            = true;
    real _object_scale    = 0.1;
    real _point_size      = 2;
    TransformMatrix3D _world_to_robot_origin = TransformMatrix3D::Identity();
    TransformMatrix3D _rotation_robot_view   = TransformMatrix3D::Identity();
    const std::string _window_name;
  };

}
