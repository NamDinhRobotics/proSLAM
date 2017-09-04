#pragma once
#include <queue>
#include "aligners/xyz_aligner.h"
#include "relocalization/local_map_correspondence.h"

namespace proslam {

//ds this class computes potential loop closures for a given local map query (the extent of computed detail can be steered easily using the different methods)
class Relocalizer {

//ds exported types
public:

  //ds augmented container, wrapping a local map with additional information required by the relocalization module
  struct Query {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Query(const LocalMap* local_map_): local_map(local_map_),
                                       matchables(local_map_->appearances()),
                                       hbst_tree(new HBSTTree(local_map_->identifier(), matchables)) {}
    ~Query() {delete hbst_tree;}

    const LocalMap* local_map;
    const HBSTNode::MatchableVector matchables;
    const HBSTTree* hbst_tree;
  };

//ds object management
PROSLAM_MAKE_PROCESSING_CLASS(Relocalizer)

//ds functionality
public:

  //ds initialize relocalization module for a new local map
  void initialize(const LocalMap* local_map_);

  //ds retrieve loop closure candidates for the given cloud
  void detect();

  //ds geometric verification and determination of spatial relation between set closures
  void compute();

  //ds integrate frame into loop closing pool
  void train();

//ds getters/setters
public:

  inline const ClosurePointerVector& closures() const {return _closures;}
  void setClosures(ClosurePointerVector& closures_) {_closures = closures_;}
  void clear();
  XYZAligner* aligner() {return _aligner;}

//ds helpers
protected:

  //ds retrieve correspondences from matches
  inline const LandmarkCorrespondence* _getCorrespondenceNN(const LandmarkCorrespondence::MatchPointerVector& matches_);

protected:

  //ds currently found closures
  ClosurePointerVector _closures;

  //ds active query for closure search
  Query* _query;

  //ds intermediate buffer for query frames before they enter the history
  std::queue<Query*> _query_history_queue;

  //ds frame-wise descriptor point clouds
  std::vector<Query*> _query_history;

  //ds correspondence retrieval
  std::set<Identifier> _mask_id_references_for_correspondences;

  //ds local map to local map alignment
  XYZAligner* _aligner = 0;

private:

  //ds module time consumption
  CREATE_CHRONOMETER(overall)
};
}
