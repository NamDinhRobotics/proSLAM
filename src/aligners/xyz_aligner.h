#pragma once
#include "base_local_map_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for camera centric point clouds (used to compute the spatial relation between local maps for a loop closure)
  class XYZAligner: public BaseLocalMapAligner, public AlignerWorkspace<6,3> {
    
  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    XYZAligner(AlignerParameters* parameters_): BaseLocalMapAligner(parameters_) {}
    ~XYZAligner() {}

  //ds functionality
  public:

    //ds initialize aligner with minimal entity
    virtual void initialize(LocalMapCorrespondence* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());

    //ds linearize the system: to be called inside oneRound
    virtual void linearize(const bool& ignore_outliers_);

    //ds solve alignment problem for one round: to be called inside converge
    virtual void oneRound(const bool& ignore_outliers_);

    //ds solve alignment problem until convergence is reached
    virtual void converge();
  };
}
