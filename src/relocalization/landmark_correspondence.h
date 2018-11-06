#pragma once
#include "types/landmark.h"

namespace proslam {

//ds container class for a single correspondence pair (produced by the relocalization module)
class LandmarkCorrespondence {

//ds exported types
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds match information between two landmark states
  struct Match {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Match(Landmark* landmark_query_,
          Landmark* landmark_reference_,
          const Count& matching_distance_hamming_): query(landmark_query_),
                                                    reference(landmark_reference_),
                                                    matching_distance_hamming(matching_distance_hamming_) {}

    Landmark* query;
    Landmark* reference;
    const Count matching_distance_hamming;
  };
  typedef std::vector<const Match*> MatchPointerVector;
  typedef std::pair<const Identifier, MatchPointerVector> MatchMapElement;
  typedef std::map<const Identifier, MatchPointerVector, std::less<Identifier>, Eigen::aligned_allocator<MatchMapElement>> MatchMap;

//ds object handling
public:

  //ds ctor
  LandmarkCorrespondence(Landmark* landmark_query_,
                         Landmark* landmark_reference_,
                         const Count& matching_count_,
                         const real& matching_ratio_): query(landmark_query_),
                                                       reference(landmark_reference_),
                                                       matching_count(matching_count_),
                                                       matching_ratio(matching_ratio_) {}

  //ds prohibit default construction
  LandmarkCorrespondence() = delete;

//ds attributes
public:

  Landmark* query;
  Landmark* reference;
  const Count matching_count;
  const real matching_ratio;

  //ds determined as inlier in registration algorithm (e.g. ICP)
  bool is_inlier = false;
};

typedef std::vector<LandmarkCorrespondence*> CorrespondencePointerVector;
}
