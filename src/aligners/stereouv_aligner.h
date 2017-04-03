#pragma once
#include "types/frame.h"
#include "base_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for pose optimization by minimizing the reprojection errors in the image plane (used to determine the robots odometry)
  class StereoUVAligner: public BaseAligner6_4 {

    //ds object handling
    public:

      StereoUVAligner(): BaseAligner6_4(1e-3, 9, 1, 1e3) {}
      ~StereoUVAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity
      void init(Frame* context_, const TransformMatrix3D& robot_to_world_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      virtual void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      virtual void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      virtual void converge();

    //ds getters/setters
    public:

      const TransformMatrix3D& robotToWorld() const {return _robot_to_world;}
      const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}
      void setWeightFramepoint(const real& weight_framepoint_) {_weight_framepoint = weight_framepoint_;}
      void setMaximumDepthNearMeters(const real& maximum_depth_near_meters_) {_maximum_depth_near_meters = maximum_depth_near_meters_;}
      void setMaximumDepthFarMeters(const real& maximum_depth_far_meters_) {_maximum_depth_far_meters = maximum_depth_far_meters_;}

    //ds aligner specific
    protected:

      //ds alignment context
      Frame* _context = 0;

      //ds objective
      TransformMatrix3D _world_to_camera_left = TransformMatrix3D::Identity();
      TransformMatrix3D _camera_left_to_world = TransformMatrix3D::Identity();

      //ds buffers
      TransformMatrix3D _world_to_robot         = TransformMatrix3D::Identity();
      TransformMatrix3D _robot_to_world         = TransformMatrix3D::Identity();
      ProjectionMatrix _projection_matrix_left  = ProjectionMatrix::Zero();
      ProjectionMatrix _projection_matrix_right = ProjectionMatrix::Zero();
      Count _image_rows                         = 0;
      Count _image_cols                         = 0;

      //ds others - think about wrapping these into the base aligner
      real _weight_framepoint         = 1;
      real _maximum_depth_near_meters = 0;
      real _maximum_depth_far_meters  = 0;
  };
}
