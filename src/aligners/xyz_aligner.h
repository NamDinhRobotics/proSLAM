#pragma once
#include "base_cloud_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for camera centric point clouds (used to compute the spatial relation between local maps for a loop closure)
  class XYZAligner: public BaseCloudAligner, public AlignerWorkspace<6,3> {

    //ds object handling
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
      XYZAligner(): BaseCloudAligner(1e-5, 0.05) {}
      ~XYZAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity
      virtual void init(Closure* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      void converge();
  };
}
