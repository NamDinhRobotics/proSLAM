#pragma once
#include "landmark_item.h"

namespace proslam {

  class Correspondence {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

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
    const real matching_ratio       = 0.0;
  };

  typedef std::vector<const Correspondence*> CorrespondencePointerVector;
} //namespace gslam
