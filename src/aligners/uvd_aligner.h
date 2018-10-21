#pragma once
#include "base_frame_aligner.h"

namespace proslam {

//ds this class specifies an aligner for pose optimization by minimizing the reprojection errors in the image plane (used to determine the robots odometry)
class UVDAligner: public BaseFrameAligner, public AlignerWorkspace<6,3> {

//ds object handling
PROSLAM_MAKE_PROCESSING_SUBCLASS(UVDAligner, AlignerParameters)

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
  CameraMatrix _camera_matrix = CameraMatrix::Zero();
};
}
