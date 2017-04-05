#pragma once
#include "correspondence.h"

namespace proslam {

  //ds this class is a container for all correspondence objects between 2 local maps (produced by the relocalization module)
  class Closure {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds object handling
  public:

    //ds ctor
    Closure(const LocalMap* local_map_query_,
            const LocalMap* local_map_reference_,
            const Count& absolute_number_of_matches_,
            const real& relative_number_of_matches_,
            const Correspondence::MatchMap& matches_,
            const CorrespondencePointerVector& correspondences_);

    //ds dtor
    ~Closure();

  //ds attributes
  public:

    const LocalMap* local_map_query;
    const LocalMap* local_map_reference;
    const Identifier identifier_query;
    const Identifier identifier_reference;
    const Count absolute_number_of_matches;
    const real relative_number_of_matches;
    const Correspondence::MatchMap matches_per_point;
    CorrespondencePointerVector correspondences;
    TransformMatrix3D transform_frame_query_to_frame_reference = TransformMatrix3D::Identity();
    real icp_inlier_ratio          = 0;
    Count icp_number_of_iterations = 0;
    Count icp_number_of_inliers    = 0;
    bool is_valid                  = false;
  };

  typedef std::vector<Closure*> ClosurePointerVector;
}
