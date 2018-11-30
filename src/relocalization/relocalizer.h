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
  void detect(const LocalMap* local_map_query_);

  //ds geometric verification and determination of spatial relation between set closures
  void compute();

//ds getters/setters
public:

  inline const ClosurePointerVector& closures() const {return _closures;}
  void setClosures(ClosurePointerVector& closures_) {_closures = closures_;}
  void clear();
  XYZAligner* aligner() {return _aligner;}

//ds helpers
protected:

  //ds retrieve correspondences from matches
  inline LandmarkCorrespondence* _getCorrespondenceNN(const LandmarkCorrespondence::MatchVector& matches_);

protected:

  //ds buffer of found closures (last compute call)
  ClosurePointerVector _closures;

  //ds local map to local map alignment
  XYZAligner* _aligner = nullptr;

  //ds added local maps (in order of calls)
  ConstLocalMapPointerVector _added_local_maps;

  //ds database of visited places (= local maps), storing a descriptor vector for each place
  HBSTTree _database;

  //ds correspondence retrieval buffer
  std::set<Identifier> _mask_id_references_for_correspondences;

private:

  CREATE_CHRONOMETER(overall)

};
}
