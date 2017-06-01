#pragma once
#include "base_frame_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for pose optimization by minimizing the reprojection errors in the image plane (used to determine the robots odometry)
  class StereoUVAligner: public BaseFrameAligner, public AlignerWorkspace<6,4> {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoUVAligner() {}
    ~StereoUVAligner() {}

  //ds functionality
  public:

    //ds initialize aligner with minimal entity
    virtual void initialize(Frame* frame_, const TransformMatrix3D& robot_to_world_ = TransformMatrix3D::Identity());

    //ds linearize the system: to be called inside oneRound
    virtual void linearize(const bool& ignore_outliers_);

    //ds solve alignment problem for one round: to be called inside converge
    virtual void oneRound(const bool& ignore_outliers_);

    //ds solve alignment problem until convergence is reached
    virtual void converge();

  //ds aligner specific
  protected:

    //ds buffers
    ProjectionMatrix _projection_matrix_left  = ProjectionMatrix::Zero();
    ProjectionMatrix _projection_matrix_right = ProjectionMatrix::Zero();
  };
}
