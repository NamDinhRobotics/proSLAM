#pragma once
#include "types/landmark_item.h"

namespace proslam {

  class Correspondence {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    struct Match {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      Match(const LandmarkItem* item_query_,
            const LandmarkItem* item_reference_,
            const Count& matching_distance_hamming_): item_query(item_query_),
                                                      item_reference(item_reference_),
                                                      matching_distance_hamming(matching_distance_hamming_) {}

      Match(const Match* match_): item_query(match_->item_query),
                                  item_reference(match_->item_reference),
                                  matching_distance_hamming(match_->matching_distance_hamming) {}

      const LandmarkItem* item_query        = 0;
      const LandmarkItem* item_reference    = 0;
      const Count matching_distance_hamming = 0;
    };
    typedef std::vector<const Match*> MatchPtrVector;
    typedef std::map<const Identifier, MatchPtrVector> MatchMap;
    typedef std::pair<const Identifier, MatchPtrVector> MatchMapElement;

  //ds object handling
  public:

    //ds ctor
    Correspondence(const LandmarkItem* item_query_,
                   const LandmarkItem* item_reference_,
                   const Count& matching_count_,
                   const real& matching_ratio_): item_query(item_query_),
                                                    item_reference(item_reference_),
                                                    matching_count(matching_count_),
                                                    matching_ratio(matching_ratio_) {}

    //ds copy ctor
    Correspondence(const Correspondence* correspondence_): item_query(correspondence_->item_query),
                                                           item_reference(correspondence_->item_reference),
                                                           matching_count(correspondence_->matching_count),
                                                           matching_ratio(correspondence_->matching_ratio) {}

  //ds public members
  public:

    const LandmarkItem* item_query     = 0;
    const LandmarkItem* item_reference = 0;
    const Count matching_count         = 0;
    const real matching_ratio          = 0.0;
  };

  typedef std::vector<const Correspondence*> CorrespondencePointerVector;
}
