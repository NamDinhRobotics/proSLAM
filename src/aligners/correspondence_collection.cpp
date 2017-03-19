#include "correspondence_collection.h"

#include "types/local_map.h"

namespace proslam {

  CorrespondenceCollection::CorrespondenceCollection(const LocalMap* keyframe_query_,
                                                     const LocalMap* keyframe_reference_,
                                                     const Count& absolute_number_of_matches_,
                                                     const real& relative_number_of_matches_,
                                                     const Correspondence::MatchMap& matches_,
                                                     const CorrespondencePointerVector& correspondences_): local_map_query(keyframe_query_),
                                                                                                           local_map_reference(keyframe_reference_),
                                                                                                           id_query(keyframe_query_->index()),
                                                                                                           id_reference(keyframe_reference_->index()),
                                                                                                           absolute_number_of_matches(absolute_number_of_matches_),
                                                                                                           relative_number_of_matches(relative_number_of_matches_),
                                                                                                           matches_per_point(matches_),
                                                                                                           correspondences(correspondences_) {}

  CorrespondenceCollection::CorrespondenceCollection(CorrespondenceCollection* collection_): local_map_query(collection_->local_map_query),
                                                                                             local_map_reference(collection_->local_map_reference),
                                                                                             id_query(collection_->local_map_query->index()),
                                                                                             id_reference(collection_->local_map_reference->index()),
                                                                                             absolute_number_of_matches(collection_->absolute_number_of_matches),
                                                                                             relative_number_of_matches(collection_->relative_number_of_matches),
                                                                                             matches_per_point(CorrespondenceCollection::getClone(collection_->matches_per_point)),
                                                                                             correspondences(CorrespondenceCollection::getClone(collection_->correspondences)) {}

  CorrespondenceCollection::~CorrespondenceCollection() {
    for (const Correspondence::MatchMapElement matches_element: matches_per_point) {
      for(const Correspondence::Match* match: matches_element.second) {
        delete match;
      }
    }
    for (const Correspondence* correspondence: correspondences) {
      delete correspondence;
    }
    correspondences.clear();
  }

  //ds deep context merging (old element can be deleted)
  void CorrespondenceCollection::absorb(const CorrespondenceCollection* collection_) {
    const CorrespondencePointerVector correspondences_copy(CorrespondenceCollection::getClone(collection_->correspondences));
    correspondences.insert(correspondences.end(), correspondences_copy.begin(), correspondences_copy.end());

    //ds check if absorbed collection has a better transform estimate
    if (icp_number_of_inliers < collection_->icp_number_of_inliers && icp_inlier_ratio < collection_->icp_inlier_ratio) {
      icp_number_of_inliers                    = collection_->icp_number_of_inliers;
      icp_inlier_ratio                         = collection_->icp_inlier_ratio;
      transform_frame_query_to_frame_reference = collection_->transform_frame_query_to_frame_reference;
    }
  }
}
