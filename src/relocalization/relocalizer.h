#pragma once
#include "aligners/xyz_aligner.h"
#include "closure.h"

namespace proslam {

//ds this class computes potential loop closures for a given local map query (the extent of computed detail can be steered easily using the different methods)
class Relocalizer {

//ds object management
PROSLAM_MAKE_PROCESSING_CLASS(Relocalizer)

//ds interface
public:

  //ds retrieve loop closure candidates for the given local map, containing descriptors for its landmarks
  void detectClosures(LocalMap* local_map_query_);

  //ds geometric verification and determination of spatial relation between closure set
  void registerClosures();

  //ds clear currently available closure buffer
  void clear();

  //! @brief keeps only a single closure, based on the maximum relative number of correspodences TODO add proper constraints
  void prune();

//ds getters/setters
public:

  inline const ClosurePointerVector& closures() const {return _closures;}
  XYZAlignerPtr aligner() {return _aligner;}

//ds helpers
protected:

  //ds retrieve correspondences from matches
  inline Closure::Correspondence* _getCorrespondenceNN(const Closure::CandidateVector& matches_);

protected:

  //ds buffer of found closures (last compute call)
  ClosurePointerVector _closures;

  //ds local map to local map alignment
  XYZAlignerPtr _aligner = nullptr;

  //ds database of visited places (= local maps), storing a descriptor vector for each place
  HBSTTree _place_database;

  //ds local maps that have been added to the place database (in order of calls)
  ConstLocalMapPointerVector _added_local_maps;

  //ds correspondence retrieval buffer
  std::set<Identifier> _mask_id_references_for_correspondences;

private:

  CREATE_CHRONOMETER(overall)

};
}
