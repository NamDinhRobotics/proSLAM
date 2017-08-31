#pragma once
#include "landmark_correspondence.h"
#include "types/local_map.h"

namespace proslam {

  //ds this class is a container for all correspondence objects between 2 local maps (produced by the relocalization module)
  class LocalMapCorrespondence {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds ctor
  LocalMapCorrespondence(const LocalMap* local_map_query_,
                         const LocalMap* local_map_reference_,
                         const Count& absolute_number_of_matches_,
                         const real& relative_number_of_matches_,
                         const LandmarkCorrespondence::MatchMap& matches_,
                         const CorrespondencePointerVector& correspondences_): local_map_query(local_map_query_),
                                                                               local_map_reference(local_map_reference_),
                                                                               identifier_query(local_map_query_->identifier()),
                                                                               identifier_reference(local_map_reference_->identifier()),
                                                                               absolute_number_of_matches(absolute_number_of_matches_),
                                                                               relative_number_of_matches(relative_number_of_matches_),
                                                                               matches_per_point(matches_),
                                                                               correspondences(correspondences_) {}

  //ds dtor
  ~LocalMapCorrespondence() {
    for (const LandmarkCorrespondence::MatchMapElement matches_element: matches_per_point) {
      for(const LandmarkCorrespondence::Match* match: matches_element.second) {
        delete match;
      }
    }
    for (const LandmarkCorrespondence* correspondence: correspondences) {
      delete correspondence;
    }
    correspondences.clear();
  }

  //ds attributes
  public:

    const LocalMap* local_map_query;
    const LocalMap* local_map_reference;
    const Identifier identifier_query;
    const Identifier identifier_reference;
    const Count absolute_number_of_matches;
    const real relative_number_of_matches;
    const LandmarkCorrespondence::MatchMap matches_per_point;
    CorrespondencePointerVector correspondences;
    TransformMatrix3D query_to_reference = TransformMatrix3D::Identity();
    real icp_inlier_ratio          = 0;
    Count icp_number_of_iterations = 0;
    Count icp_number_of_inliers    = 0;
    bool is_valid                  = false;
  };

  typedef std::vector<LocalMapCorrespondence*> ClosurePointerVector;
}
