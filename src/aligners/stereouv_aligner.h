#pragma once
#include "base_frame_aligner.h"

namespace proslam {

//ds this class specifies an aligner for pose optimization by minimizing the reprojection errors in the image plane (used to determine the robots odometry)
class StereoUVAligner: public BaseFrameAligner, public AlignerWorkspace<6, 4> {

//ds object handling
PROSLAM_MAKE_PROCESSING_SUBCLASS(StereoUVAligner, AlignerParameters)

//ds functionality
public:

  //ds initialize aligner with minimal entity
  virtual void initialize(const Frame* frame_previous_,
                          const Frame* frame_current_,
                          const TransformMatrix3D& previous_to_current_);

  //ds linearize the system: to be called inside oneRound
  virtual void linearize(const bool& ignore_outliers_);

  //ds solve alignment problem for one round: to be called inside converge
  virtual void oneRound(const bool& ignore_outliers_);

  //ds solve alignment problem until convergence is reached
  virtual void converge();

//ds aligner specific
protected:

  //ds buffers
  CameraMatrix _camera_calibration_matrix   = CameraMatrix::Zero();
  Vector3 _offset_camera_right              = Vector3::Zero();

  //ds 3D points in camera frame
  std::vector<Vector3, Eigen::aligned_allocator<Vector3> > _moving;
  std::vector<real> _weights_translation;
};
}
