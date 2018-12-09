#pragma once
#include "types/local_map.h"

namespace proslam {

//ds this class is a container for all correspondence objects between 2 local maps (produced by the relocalization module)
class Closure {

//ds exported types
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds match information between two landmark states
  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Candidate(Landmark* landmark_query_,
              Landmark* landmark_reference_,
              const Count& matching_distance_hamming_): query(landmark_query_),
                                                        reference(landmark_reference_),
                                                        matching_distance_hamming(matching_distance_hamming_) {}

    Landmark* query;
    Landmark* reference;
    const Count matching_distance_hamming;
  };

  typedef std::vector<Candidate, Eigen::aligned_allocator<Candidate>> CandidateVector;
  typedef std::pair<const Identifier, CandidateVector> CandidateMapElement;
  typedef std::map<const Identifier, CandidateVector, std::less<Identifier>, Eigen::aligned_allocator<CandidateMapElement>> CandidateMap;

  //ds container for a single correspondence pair (produced by the relocalization module)
  struct Correspondence {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Correspondence(Landmark* landmark_query_,
                   Landmark* landmark_reference_,
                   const Count& matching_count_,
                   const real& matching_ratio_): query(landmark_query_),
                                                 reference(landmark_reference_),
                                                 matching_count(matching_count_),
                                                 matching_ratio(matching_ratio_) {}

    Landmark* query;
    Landmark* reference;
    const Count matching_count;
    const real matching_ratio;

    //ds determined as inlier in registration algorithm (e.g. ICP)
    bool is_inlier = false;
  };

  typedef std::vector<Correspondence*> CorrespondencePointerVector;

  //ds loop closure constraint element between 2 local maps
  struct ClosureConstraint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ClosureConstraint(LocalMap* local_map_,
                      const TransformMatrix3D& relation_,
                      const Closure::CorrespondencePointerVector& landmark_correspondences_,
                      const real& omega_ = 1): local_map(local_map_),
                                               relation(relation_),
                                               landmark_correspondences(landmark_correspondences_),
                                               omega(omega_){}

    LocalMap* local_map;
    const TransformMatrix3D relation;
    const Closure::CorrespondencePointerVector landmark_correspondences;
    const real omega;
  };

  typedef std::vector<ClosureConstraint, Eigen::aligned_allocator<ClosureConstraint>> ClosureConstraintVector;

  //ds landmark snapshot at creation of local map
  struct LandmarkState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LandmarkState(Landmark* landmark_,
                  PointCoordinates coordinates_in_local_map_): landmark(landmark_),
                                                               coordinates_in_local_map(coordinates_in_local_map_) {}

    Landmark* landmark;
    PointCoordinates coordinates_in_local_map;
  };

  typedef std::pair<const Identifier, LandmarkState> LandmarkStateMapElement;
  typedef std::map<const Identifier, LandmarkState, std::less<Identifier>, Eigen::aligned_allocator<LandmarkStateMapElement> > LandmarkStateMap;

//ds object life
public:

//ds ctor
Closure(LocalMap* local_map_query_,
        LocalMap* local_map_reference_,
        const Count& absolute_number_of_matches_,
        const real& relative_number_of_matches_,
        const CorrespondencePointerVector& correspondences_): local_map_query(local_map_query_),
                                                              local_map_reference(local_map_reference_),
                                                              absolute_number_of_matches(absolute_number_of_matches_),
                                                              relative_number_of_matches(relative_number_of_matches_),
                                                              correspondences(correspondences_) {}

//ds dtor
~Closure() {

  //ds if closure is invalidated it has not been consumed by the world map and is to be freed
  if (!is_valid) {
    for (const Correspondence* correspondence: correspondences) {
      delete correspondence;
    }
  }
  correspondences.clear();
}

//ds attributes
public:

  LocalMap* local_map_query;
  LocalMap* local_map_reference;
  const Count absolute_number_of_matches;
  const real relative_number_of_matches;
  CorrespondencePointerVector correspondences;

  //ds attributes set after registration
  TransformMatrix3D query_to_reference = TransformMatrix3D::Identity();
  real icp_inlier_ratio          = 0;
  Count icp_number_of_iterations = 0;
  Count icp_number_of_inliers    = 0;
  bool is_valid                  = false;

};

typedef std::vector<Closure*> ClosurePointerVector;

}
