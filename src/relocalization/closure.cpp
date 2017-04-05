#include "closure.h"

#include "types/local_map.h"

namespace proslam {

  //ds ctor
  Closure::Closure(const LocalMap* keyframe_query_,
                   const LocalMap* keyframe_reference_,
                   const Count& absolute_number_of_matches_,
                   const real& relative_number_of_matches_,
                   const Correspondence::MatchMap& matches_,
                   const CorrespondencePointerVector& correspondences_): local_map_query(keyframe_query_),
                                                                         local_map_reference(keyframe_reference_),
                                                                         identifier_query(keyframe_query_->identifier()),
                                                                         identifier_reference(keyframe_reference_->identifier()),
                                                                         absolute_number_of_matches(absolute_number_of_matches_),
                                                                         relative_number_of_matches(relative_number_of_matches_),
                                                                         matches_per_point(matches_),
                                                                         correspondences(correspondences_) {}

  //ds dtor
  Closure::~Closure() {
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
}
