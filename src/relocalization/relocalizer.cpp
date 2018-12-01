#include "relocalizer.h"

namespace proslam {

Relocalizer::Relocalizer(RelocalizerParameters* parameters_): _parameters(parameters_) {
  _added_local_maps.clear();
  clear();
  LOG_INFO(std::cerr << "Relocalizer::Relocalizer|constructed" << std::endl)
}

void Relocalizer::configure() {
  LOG_INFO(std::cerr << "Relocalizer::configure|configuring" << std::endl)
  _added_local_maps.clear();
  clear();

  //ds allocate and configure aligner unit
  _aligner = XYZAlignerPtr(new XYZAligner(_parameters->aligner));
  _aligner->configure();
  LOG_INFO(std::cerr << "Relocalizer::configure|configured" << std::endl)
}

Relocalizer::~Relocalizer() {
  LOG_INFO(std::cerr << "Relocalizer::~Relocalizer|destroying" << std::endl)
  _added_local_maps.clear();
  clear();
  LOG_INFO(std::cerr << "Relocalizer::~Relocalizer|destroyed" << std::endl)
}

//ds retrieve loop closure candidates for the given cloud
void Relocalizer::detectClosures(const LocalMap* local_map_query_) {
  CHRONOMETER_START(overall)
  if (!local_map_query_) {
    return;
  }

  //ds always add the entry (only matching is optional)
  _added_local_maps.push_back(local_map_query_);

  //ds if we are not yet in query range - only add matchables and nothing else to do
  if (_place_database.size() < _parameters->preliminary_minimum_interspace_queries) {

    //ds add matchables
    _place_database.add(local_map_query_->appearances(), srrg_hbst::SplittingStrategy::SplitEven);
    return;
  }

  //ds matching result container: a map that contains a vector of matches to the current image for each reference image
  HBSTTree::MatchVectorMap matches_per_reference_image;

  //ds query database for current matchables and integrate current image simultaneously
  _place_database.matchAndAdd(local_map_query_->appearances(), matches_per_reference_image, _parameters->maximum_descriptor_distance);

  //ds evaluate matches for each reference image in the range
  const Count number_of_query_matchables = local_map_query_->appearances().size();
  const Count maximum_index_reference    = _place_database.size()-_parameters->preliminary_minimum_interspace_queries;
  for (Count index_reference_local_map = 0; index_reference_local_map < maximum_index_reference; ++index_reference_local_map) {
    HBSTTree::MatchVector& multiple_matches_mixed = matches_per_reference_image.at(index_reference_local_map);

    //ds compute relative matching ratio (how many of the query matchables were matched)
    const real relative_number_of_matches = static_cast<real>(multiple_matches_mixed.size())/number_of_query_matchables;

    //ds skip this reference image if matching ratio is insufficient
    if (relative_number_of_matches < _parameters->preliminary_minimum_matching_ratio) {
      continue;
    }

    //ds loop over all matches to organize them per landmark
    Closure::CandidateMap multiple_matches_per_landmark;
    for (const HBSTTree::Match& match: multiple_matches_mixed) {

      //ds buffer landmark identifier
      Landmark* landmark_query                    = match.object_query;
      Landmark* landmark_reference                = match.object_reference;
      const Identifier& query_landmark_identifier = landmark_query->identifier();

      //ds update match map (adding a new entry if not existing yet)
      try {

        //ds add a new match to the given query point
        multiple_matches_per_landmark.at(query_landmark_identifier).push_back(Closure::Candidate(landmark_query, landmark_reference, match.distance));
      } catch(const std::out_of_range& /*exception*/) {

        //ds initialize the first match for the given query point
        multiple_matches_per_landmark.insert(std::make_pair(query_landmark_identifier,
                                                            Closure::CandidateVector(1, Closure::Candidate(landmark_query, landmark_reference, match.distance))));
      }
    }

    //ds skip further processing if number of matching landmarks is insufficient
    if (multiple_matches_per_landmark.size() < _parameters->minimum_number_of_matched_landmarks) {
      continue;
    }

    //ds prepare point to point correspondence search
    Closure::CorrespondencePointerVector correspondences;
    _mask_id_references_for_correspondences.clear();

    //ds compute the best point to point correspondences from multiple match candidates
    for(const Closure::CandidateMapElement& multiple_matches: multiple_matches_per_landmark) {

      //ds retrieve best correspondence for the multiple matches
      Closure::Correspondence* correspondence = _getCorrespondenceNN(multiple_matches.second);
      if (correspondence) {
        correspondences.push_back(correspondence);
      }
    }

    //ds add to closure buffer
    _closures.push_back(new Closure(local_map_query_,
                                    _added_local_maps[index_reference_local_map],
                                    multiple_matches_per_landmark.size(),
                                    relative_number_of_matches,
                                    correspondences));
  }
  CHRONOMETER_STOP(overall)
  return;
}

//ds geometric verification and determination of spatial relation between a set of closures
void Relocalizer::registerClosures() {
  CHRONOMETER_START(overall)
  for(Closure* closure: _closures) {
    _aligner->initialize(closure);
    _aligner->converge();
  }
  CHRONOMETER_STOP(overall)
}

void Relocalizer::clear() {
  CHRONOMETER_START(overall)
  for(const Closure* closure: _closures) {
    delete closure;
  }
  _closures.clear();
  _mask_id_references_for_correspondences.clear();
  CHRONOMETER_STOP(overall)
}

//ds retrieve correspondences from matches
Closure::Correspondence* Relocalizer::_getCorrespondenceNN(const Closure::CandidateVector& matches_) {
  assert(0 < matches_.size());

  //ds point counts
  std::multiset<Count> counts;

  //ds best match and count so far
  const Closure::Candidate* match_best = nullptr;
  Count count_best = 0;

  //ds loop over the list and count entries
  for (const Closure::Candidate& match: matches_) {

    //ds update count - if not in the mask
    if (0 == _mask_id_references_for_correspondences.count(match.reference->identifier())) {
      counts.insert(match.reference->identifier());
      const Count count_current = counts.count(match.reference->identifier());

      //ds if we get a better count
      if (count_best < count_current) {
        count_best = count_current;
        match_best = &match;
      }
    }
  }

  //ds if a match was found with sufficient confidence
  if (match_best && count_best > _parameters->minimum_matches_per_correspondence) {

    //ds block matching against this point by adding it to the mask
    _mask_id_references_for_correspondences.insert(match_best->reference->identifier());

    //ds return the found correspondence
    return new Closure::Correspondence(match_best->query,
                                       match_best->reference,
                                       count_best, static_cast<real>(count_best)/matches_.size());
  }

  //ds no match was found
  return nullptr;
}
}
