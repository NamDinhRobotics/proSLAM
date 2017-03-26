#include "relocalizer.h"

#include "srrg_types/types.hpp"

namespace proslam {
  using namespace srrg_core;

  Relocalizer::Relocalizer(): _query(0), _aligner(new XYZAligner()) {
    std::cerr << "Relocalizer::Relocalizer|constructing" << std::endl;
    clear();
    _query_history.clear();
    assert(_query_history_queue.empty());
    assert(_aligner != 0);
    std::cerr << "Relocalizer::Relocalizer|constructed" << std::endl;
  }

  Relocalizer::~Relocalizer() {
    std::cerr << "Relocalizer::Relocalizer|destroying" << std::endl;

    //ds free closure buffer
    clear();

    //ds free history
    for (const Query* query: _query_history) {
      for (const HBSTMatchable* matchable: query->matchables) {
        delete matchable;
      }
      delete query;
    }
    _query_history.clear();

    //ds free history buffer
    for (Index i = 0; i < _query_history_queue.size(); ++i) {
      const Query* query = _query_history_queue.front();
      _query_history_queue.pop();
      for (const HBSTMatchable* matchable: query->matchables) {
        delete matchable;
      }
      delete query;
    }

    //ds free active query - if available
    if (_query) {
      delete _query;
    }

    std::cerr << "Relocalizer::Relocalizer|destroyed" << std::endl;
  }

  void Relocalizer::init(const LocalMap* keyframe) {
    CHRONOMETER_START(overall)
    _query = new Query(keyframe);
    CHRONOMETER_STOP(overall)
  }

  //ds integrate frame into loop closing pool
  void Relocalizer::train() {
    CHRONOMETER_START(overall)

    //ds if query is valid
    if (0 != _query && 0 < _query->appearances.size()) {

      //ds add the active query to our database structure
      _query_history_queue.push(_query);

      //ds check if we can pop the first element of the buffer into our history
      if (_query_history_queue.size() > _preliminary_minimum_interspace_queries) {
        _query_history.push_back(_query_history_queue.front());
        _query_history_queue.pop();
      }
    }

    //ds reset handles
    _query = 0;
    CHRONOMETER_STOP(overall)
  }

  void Relocalizer::flush() {
    CHRONOMETER_START(overall)

    //ds for all elements in the current history
    while (_query_history_queue.size() > 0) {
      _query_history.push_back(_query_history_queue.front());
      std::cerr << "flushed to history: " << _query_history_queue.front()->keyframe->identifier() << std::endl;
      _query_history_queue.pop();
    }

    CHRONOMETER_STOP(overall)
  }

  //ds retrieve loop closure candidates for the given cloud
  void Relocalizer::detect(const bool& force_matching_) {
    CHRONOMETER_START(overall)

    //ds clear output buffers
    clear();

    //ds evaluate all past queries
    for (const Query* reference: _query_history) {

      //ds get match count
      const real matching_ratio = reference->hbst_tree->getMatchingRatioFlat(_query->matchables);

      //ds if acceptable
      if (matching_ratio > _preliminary_minimum_matching_ratio || force_matching_) {
        assert(reference != 0);

        //ds matches within the current reference
        HBSTTree::MatchVector matches_unfiltered;
        Correspondence::MatchMap descriptor_matches_pointwise;

        //ds get matches
        assert(0 < _query->matchables.size());
        assert(0 < reference->matchables.size());
        reference->hbst_tree->match(_query->matchables, matches_unfiltered);
        assert(0 < matches_unfiltered.size());
        const Count absolute_number_of_descriptor_matches = matches_unfiltered.size();

        //ds loop over all matches
        for (const HBSTTree::Match match: matches_unfiltered) {

          const Landmark::Appearance* appearance_query     = _query->appearances[match.identifier_query];
          const Landmark::Appearance* appearance_reference = reference->appearances[match.identifier_reference];
          const Identifier& query_index                    = appearance_query->landmark_state->landmark->identifier();

          try{

            //ds add a new match to the given query point
            descriptor_matches_pointwise.at(query_index).push_back(new Correspondence::Match(appearance_query->landmark_state,
                                                                   appearance_reference->landmark_state,
                                                                   match.distance));
          } catch(const std::out_of_range& /*exception*/) {

            //ds initialize the first match for the given query point
            descriptor_matches_pointwise.insert(std::make_pair(query_index, Correspondence::MatchPtrVector(1, new Correspondence::Match(appearance_query->landmark_state,
                                                                                              appearance_reference->landmark_state,
                                                                                              match.distance))));
          }
        }
        assert(0 < absolute_number_of_descriptor_matches);
        assert(0 < _query->appearances.size());
        assert(0 < descriptor_matches_pointwise.size());
        const real relative_number_of_descriptor_matches_query     = static_cast<real>(absolute_number_of_descriptor_matches)/_query->appearances.size();
        //const gt_real relative_number_of_descriptor_matches_reference = static_cast<gt_real>(absolute_number_of_descriptor_matches)/reference->appearances.size();
        //const gt_real relative_delta = std::fabs(relative_number_of_descriptor_matches_query-relative_number_of_descriptor_matches_reference)/relative_number_of_descriptor_matches_reference;

        //ds if the result quality is sufficient
        if (descriptor_matches_pointwise.size() > _minimum_absolute_number_of_matches_pointwise) {

          //ds correspondences
          CorrespondencePointerVector correspondences;
          _mask_id_references_for_correspondences.clear();

          //ds compute point-to-point correspondences for all matches
          for(const Correspondence::MatchMapElement matches_per_point: descriptor_matches_pointwise){
            const Correspondence* correspondence = getCorrespondenceNN(matches_per_point.second);
            if (correspondence != 0) {
              correspondences.push_back(correspondence);
            }
          }
          assert(0 < correspondences.size());

            //ds update closures
          _closures.push_back(new CorrespondenceCollection(_query->keyframe,
                                          reference->keyframe,
                                          absolute_number_of_descriptor_matches,
                                          relative_number_of_descriptor_matches_query,
                                          descriptor_matches_pointwise,
                                          correspondences));
        } /*else {
          std::cerr << _query->keyframe->index() << " | " << reference->keyframe->index() << " not enough matches: " << descriptor_matches_pointwise.size() << std::endl;
        }*/
      }
    }
    CHRONOMETER_STOP(overall)
  }

  //ds geometric verification
  void Relocalizer::compute() {
    CHRONOMETER_START(overall)
    for(CorrespondenceCollection* closure: _closures) {
      _aligner->init(closure);
      _aligner->converge();
    }
    CHRONOMETER_STOP(overall)
  }

  const Correspondence* Relocalizer::getCorrespondenceNN(const Correspondence::MatchPtrVector& matches_) {
    assert(0 < matches_.size());

    //ds point counts
    std::multiset<Count> counts;

    //ds best match and count so far
    const Correspondence::Match* match_best = 0;
    Count count_best        = 0;

    //ds loop over the list and count entries
    for(const Correspondence::Match* match: matches_){

      //ds update count - if not in the mask
      if(0 == _mask_id_references_for_correspondences.count(match->item_reference->landmark->identifier())) {
        counts.insert(match->item_reference->landmark->identifier());
        const Count count_current = counts.count(match->item_reference->landmark->identifier());

        //ds if we get a better count
        if( count_best < count_current ){
          count_best = count_current;
          match_best = match;
        }
      }
    }

    if(match_best != 0 && count_best > _minimum_matches_per_correspondence ) {

      //ds block matching against this point by adding it to the mask
      _mask_id_references_for_correspondences.insert(match_best->item_reference->landmark->identifier());

      //ds return the found correspondence
      return new Correspondence(match_best->item_query,
                                match_best->item_reference,
                                count_best, static_cast<real>(count_best)/matches_.size());
    }

    return 0;
  }

  void Relocalizer::clear() {
    for(const CorrespondenceCollection* closure: _closures) {
      delete closure;
    }
    _closures.clear();
    _mask_id_references_for_correspondences.clear();
  }
}
