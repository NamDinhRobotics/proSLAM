#include "local_map_correspondence.h"
#include "types/local_map.h"

namespace proslam {

  //ds ctor
  LocalMapCorrespondence::LocalMapCorrespondence(const LocalMap* local_map_query_,
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
  LocalMapCorrespondence::~LocalMapCorrespondence() {
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
}
