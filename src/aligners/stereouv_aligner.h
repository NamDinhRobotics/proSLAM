#pragma once
#include "base_frame_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for pose optimization by minimizing the reprojection errors in the image plane (used to determine the robots odometry)
  class StereoUVAligner: public BaseFrameAligner, public AlignerWorkspace<6,4> {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoUVAligner(): BaseFrameAligner(1e-3, 9, 1, 1e3) {}
    ~StereoUVAligner() {}

  //ds required interface
  public:
    //ds initialize aligner with minimal entity
    virtual void init(Frame* context_, const TransformMatrix3D& robot_to_world_ = TransformMatrix3D::Identity());


    //ds linearize the system: to be called inside oneRound
    virtual void linearize(const bool& ignore_outliers_);

    //ds solve alignment problem for one round: to be called inside converge
    virtual void oneRound(const bool& ignore_outliers_);

    //ds solve alignment problem until convergence is reached
    virtual void converge();

  //ds getters/setters
  public:

  //ds aligner specific
  protected:

    //ds alignment context
    Frame* _context = 0;

    //ds objective
    TransformMatrix3D _world_to_camera_left = TransformMatrix3D::Identity();
    TransformMatrix3D _camera_left_to_world = TransformMatrix3D::Identity();

    //ds buffers
    ProjectionMatrix _projection_matrix_left  = ProjectionMatrix::Zero();
    ProjectionMatrix _projection_matrix_right = ProjectionMatrix::Zero();
    Count _image_rows                         = 0;
    Count _image_cols                         = 0;
  };
}
