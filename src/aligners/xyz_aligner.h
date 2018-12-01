#pragma once
#include "base_local_map_aligner.h"

namespace proslam {

//ds this class specifies an aligner for camera centric point clouds (used to compute the spatial relation between local maps for a loop closure)
class XYZAligner: public BaseLocalMapAligner, public AlignerWorkspace<6,3> {

//ds object handling
PROSLAM_MAKE_PROCESSING_SUBCLASS(XYZAligner, AlignerParameters)

//ds functionality
public:

  //ds initialize aligner with minimal entity
  virtual void initialize(Closure* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());

  //ds linearize the system: to be called inside oneRound
  virtual void linearize(const bool& ignore_outliers_);

  //ds solve alignment problem for one round: to be called inside converge
  virtual void oneRound(const bool& ignore_outliers_);

  //ds solve alignment problem until convergence is reached
  virtual void converge();

//ds attributes
protected:

  //ds solver setup (TODO port solver)
  Count _number_of_measurements = 0;
  std::vector<DimensionMatrix, Eigen::aligned_allocator<DimensionMatrix> > _information_vector;
  std::vector<Vector3> _moving;
  std::vector<Vector3> _fixed;

};

typedef std::shared_ptr<XYZAligner> XYZAlignerPtr;

}
