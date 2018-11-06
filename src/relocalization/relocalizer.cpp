#include "relocalizer.h"

namespace proslam {

Relocalizer::Relocalizer(RelocalizerParameters* parameters_): _parameters(parameters_),
                                                              _aligner(new XYZAligner(parameters_->aligner)) {
  LOG_DEBUG(std::cerr << "Relocalizer::Relocalizer|constructed" << std::endl)
}

void Relocalizer::configure() {
  LOG_DEBUG(std::cerr << "Relocalizer::configure|configuring" << std::endl)
  clear();
  _query_history.clear();
  assert(_query_history_queue.empty());
  assert(_aligner != 0);

  //ds configure aligner unit
  _aligner->configure();
  LOG_DEBUG(std::cerr << "Relocalizer::configure|configured" << std::endl)
}

Relocalizer::~Relocalizer() {
  LOG_DEBUG(std::cerr << "Relocalizer::~Relocalizer|destroying" << std::endl)

  //ds free closure buffer
  clear();

  //ds free history
  for (const Query* query: _query_history) {
    delete query;
  }
  _query_history.clear();

  //ds free history buffer
  for (Index i = 0; i < _query_history_queue.size(); ++i) {
    const Query* query = _query_history_queue.front();
    _query_history_queue.pop();
    delete query;
  }

  //ds free active query - if available
  delete _query;

  //ds free aligner
  delete _aligner;
  LOG_DEBUG(std::cerr << "Relocalizer::~Relocalizer|destroyed" << std::endl)
}

//ds initialize relocalization module for a new local map
void Relocalizer::initialize(const LocalMap* local_map_) {
  CHRONOMETER_START(overall)
  if (_query) {
    delete _query;
  }
  _query = new Query(local_map_);
  CHRONOMETER_STOP(overall)
}

//ds retrieve loop closure candidates for the given cloud
void Relocalizer::detect() {
  CHRONOMETER_START(overall)
  if (!_query) {
    return;
  }

  //ds evaluate all past queries
  for (const Query* reference: _query_history) {

    //ds get transform prior
    const TransformMatrix3D transform_estimate_query_to_reference = reference->local_map->worldToLocalMap()*_query->local_map->localMapToWorld();

    //ds compute absolute distance
    const real distance_meters_squared = transform_estimate_query_to_reference.translation().squaredNorm();

    //ds skip the candidate if too far from the odometry
    if (distance_meters_squared > 25*25) {
      continue;
    }

    //ds matches within the current reference
    HBSTTree::MatchVector matches_unfiltered;
    LandmarkCorrespondence::MatchMap matches_per_landmark;

    //ds get matches
    assert(0 < _query->matchables.size());
    assert(0 < reference->matchables.size());
    reference->database->match(_query->matchables, matches_unfiltered);
    assert(0 < matches_unfiltered.size());
    const Count& absolute_number_of_matches = matches_unfiltered.size();
    if (static_cast<real>(absolute_number_of_matches)/_query->matchables.size() < _parameters->preliminary_minimum_matching_ratio) {
      continue;
    }

    //ds loop over all matches
    for (const HBSTTree::Match match: matches_unfiltered) {

      //ds buffer landmark identifier
      Landmark* landmark_query       = static_cast<Landmark*>(match.pointer_query);
      Landmark* landmark_reference   = static_cast<Landmark*>(match.pointer_reference);
      const Identifier& query_landmark_identifier = landmark_query->identifier();

      try{

        //ds add a new match to the given query point
        matches_per_landmark.at(query_landmark_identifier).push_back(new LandmarkCorrespondence::Match(landmark_query, landmark_reference, match.distance));
      } catch(const std::out_of_range& /*exception*/) {

        //ds initialize the first match for the given query point
        matches_per_landmark.insert(std::make_pair(query_landmark_identifier, LandmarkCorrespondence::MatchPointerVector(1, new LandmarkCorrespondence::Match(landmark_query, landmark_reference, match.distance))));
      }
    }
    assert(0 < absolute_number_of_matches);
    assert(0 < matches_per_landmark.size());
    const real relative_number_of_matches = static_cast<real>(absolute_number_of_matches)/_query->matchables.size();

    //ds if the result quality is sufficient
    if (matches_per_landmark.size() > _parameters->minimum_number_of_matches_per_landmark) {

      //ds correspondences
      CorrespondencePointerVector correspondences;
      _mask_id_references_for_correspondences.clear();

      //ds compute point-to-point correspondences for all matches
      for(const LandmarkCorrespondence::MatchMapElement matches_per_point: matches_per_landmark){
        LandmarkCorrespondence* correspondence = _getCorrespondenceNN(matches_per_point.second);
        if (correspondence != 0) {
          correspondences.push_back(correspondence);
        }
      }
      assert(0 < correspondences.size());

      //ds update closures
      _closures.push_back(new Closure(_query->local_map,
                                      reference->local_map,
                                      absolute_number_of_matches,
                                      relative_number_of_matches,
                                      matches_per_landmark,
                                      correspondences));
    }
  }
  CHRONOMETER_STOP(overall)
}

//ds geometric verification and determination of spatial relation between set closures
void Relocalizer::compute() {
  CHRONOMETER_START(overall)
  if (!_query) {
    return;
  }
  for(Closure* closure: _closures) {
    _aligner->initialize(closure);
    _aligner->converge();
  }
  CHRONOMETER_STOP(overall)
}

//ds integrate frame into loop closing pool
void Relocalizer::train() {
  CHRONOMETER_START(overall)
  if (!_query) {
    return;
  }

  //ds add the active query to our database structure
  _query_history_queue.push(_query);

  //ds check if we can pop the first element of the buffer into our history
  if (_query_history_queue.size() > _parameters->preliminary_minimum_interspace_queries) {
    _query_history.push_back(_query_history_queue.front());
    _query_history_queue.pop();
  }

  //ds reset handles
  _query = nullptr;
  CHRONOMETER_STOP(overall)
}

void Relocalizer::clear() {
  if (_query) {
    delete _query;
  }
  for(const Closure* closure: _closures) {
    delete closure;
  }
  _closures.clear();
  _mask_id_references_for_correspondences.clear();
}

//ds retrieve correspondences from matches
LandmarkCorrespondence* Relocalizer::_getCorrespondenceNN(const LandmarkCorrespondence::MatchPointerVector& matches_) {
  assert(0 < matches_.size());

  //ds point counts
  std::multiset<Count> counts;

  //ds best match and count so far
  const LandmarkCorrespondence::Match* match_best = 0;
  Count count_best = 0;

  //ds loop over the list and count entries
  for(const LandmarkCorrespondence::Match* match: matches_){

    //ds update count - if not in the mask
    if(0 == _mask_id_references_for_correspondences.count(match->reference->identifier())) {
      counts.insert(match->reference->identifier());
      const Count count_current = counts.count(match->reference->identifier());

      //ds if we get a better count
      if( count_best < count_current ){
        count_best = count_current;
        match_best = match;
      }
    }
  }

  //ds if a match was found with sufficient confidence
  if(match_best && count_best > _parameters->minimum_matches_per_correspondence ) {

    //ds block matching against this point by adding it to the mask
    _mask_id_references_for_correspondences.insert(match_best->reference->identifier());

    //ds return the found correspondence
    return new LandmarkCorrespondence(match_best->query,
                                      match_best->reference,
                                      count_best, static_cast<real>(count_best)/matches_.size());
  }

  //ds no match was found
  return nullptr;
}
}
