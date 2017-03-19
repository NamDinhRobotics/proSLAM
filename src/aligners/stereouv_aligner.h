#pragma once
#include "types/frame.h"
#include "base_aligner.h"

namespace proslam {

  class StereoUVAligner: public BaseAligner6_4 {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    public:

      //ds instantiation controlled by aligner factory
      StereoUVAligner(): BaseAligner6_4(1e-3, 9, 1, 1e3) {}
      ~StereoUVAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity TODO purify this
      virtual void init(BaseContext* context_, const TransformMatrix3D& robot_to_world_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      virtual void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      virtual void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      virtual void converge();

      //ds additional accessors
    public:

      //ds getters/setters
      const TransformMatrix3D robotToWorld() const {return _robot_to_world;}
      const TransformMatrix3D worldToRobot() const {return _world_to_robot;}
      void setWeightFramepoint(const real& weight_framepoint_) {_weight_framepoint = weight_framepoint_;}
      void setMaximumDepthClose(const real& maximum_depth_close_) {_maximum_depth_close = maximum_depth_close_;}
      void setMaximumDepthFar(const real& maximum_depth_far_) {_maximum_depth_far = maximum_depth_far_;}

    //ds aligner specific
    protected:

      //ds context
      Frame* _context = 0;

      //ds objective
      TransformMatrix3D _world_to_camera_left = TransformMatrix3D::Identity();
      TransformMatrix3D _camera_left_to_world = TransformMatrix3D::Identity();

      //ds optimization wrapping
      TransformMatrix3D _world_to_robot = TransformMatrix3D::Identity();
      TransformMatrix3D _robot_to_world = TransformMatrix3D::Identity();
      ProjectionMatrix _projection_matrix_left  = ProjectionMatrix::Zero();
      ProjectionMatrix _projection_matrix_right = ProjectionMatrix::Zero();
      Count _image_rows                         = 0;
      Count _image_cols                         = 0;

      //ds others - think about wrapping these into the factory
      TransformMatrix3D _robot_to_world_previous = TransformMatrix3D::Identity();
      real _weight_framepoint                 = 1;
      real _maximum_depth_close               = 0;
      real _maximum_depth_far                 = 0;
  };
}
