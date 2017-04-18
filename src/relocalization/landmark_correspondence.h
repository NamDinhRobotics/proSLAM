#pragma once
#include "types/landmark.h"

namespace proslam {

  //ds container class for a single correspondence pair (produced by the relocalization module)
  class LandmarkCorrespondence {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds match information between two landmark states
    struct Match {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      Match(const Landmark::State* landmark_query_,
            const Landmark::State* landmark_reference_,
            const Count& matching_distance_hamming_): query(landmark_query_),
                                                      reference(landmark_reference_),
                                                      matching_distance_hamming(matching_distance_hamming_) {}

      const Landmark::State* query;
      const Landmark::State* reference;
      const Count matching_distance_hamming;
    };
    typedef std::vector<const Match*> MatchPointerVector;
    typedef std::map<const Identifier, MatchPointerVector> MatchMap;
    typedef std::pair<const Identifier, MatchPointerVector> MatchMapElement;

  //ds object handling
  public:

    //ds ctor
    LandmarkCorrespondence(const Landmark::State* landmark_query_,
                           const Landmark::State* landmark_reference_,
                           const Count& matching_count_,
                           const real& matching_ratio_): query(landmark_query_),
                                                         reference(landmark_reference_),
                                                         matching_count(matching_count_),
                                                         matching_ratio(matching_ratio_) {}

    //ds prohibit default construction
    LandmarkCorrespondence() = delete;

  //ds attributes
  public:

    const Landmark::State* query;
    const Landmark::State* reference;
    const Count matching_count;
    const real matching_ratio;
  };

  typedef std::vector<const LandmarkCorrespondence*> CorrespondencePointerVector;
}
