#pragma once
#include "types/contexts/gt_tracking_context.h"
#include "utilities/gt_optimizer.h"

//#define ENABLE_CLOSURE_CHECKER

namespace gslam {
  class Mapper {

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mapper();
    ~Mapper();

  public:

    void closeKeyFrames(const Identifier& id_query_, const Identifier& id_reference_, const TransformMatrix3D& transform_query_to_reference_);
    void optimize(TrackingContext* context_);
    void optimizeLandmarks(TrackingContext* context_);
    void optimizePoses(TrackingContext* context_);

  protected:

    //ds optimization
    g2o::SparseOptimizer* _optimizer = 0;
    Identifier _id_last_optimization = 0;
    const Identifier _id_interspace_poses   = 10000000; //ds limit: 2147483647

#ifdef ENABLE_CLOSURE_CHECKER

    //ds closure checking
    const gt_real _closure_checker_threshold = 0.1;
    const Count _closure_checker_window      = 2;

#endif //ENABLE_CLOSURE_CHECKER

    CREATE_CHRONOMETER(overall)

  }; //class Mapper
} //namespace gtracker
