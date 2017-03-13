#pragma once
#include "../types/optimizer.h"
#include "types/contexts/world_map.h"

namespace proslam {
  class Mapper {

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mapper();
    ~Mapper();

  public:

    void closeKeyFrames(const Identifier& id_query_, const Identifier& id_reference_, const TransformMatrix3D& transform_query_to_reference_);
    void optimize(WorldMap* context_);
    void optimizeLandmarks(WorldMap* context_);
    void optimizePoses(WorldMap* context_);

  protected:

    //ds optimization
    g2o::SparseOptimizer* _optimizer = 0;
    Identifier _id_last_optimization = 0;
    const Identifier _id_interspace_poses   = 10000000; //ds limit: 2147483647

    CREATE_CHRONOMETER(overall)

  }; //class Mapper
} //namespace gtracker
