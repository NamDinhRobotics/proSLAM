#pragma once
#include <queue>
#include "aligners/xyz_aligner.h"
#include "types/local_map.h"

namespace proslam {

  //ds this class computes potential loop closures for a given local map query (the extent of computed detail can be steered easily using the different methods)
  class Relocalizer {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds augmented container, wrapping a local map with additional information required by the relocalization module
    struct Query {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      Query(const LocalMap* local_map_): local_map(local_map_),
                                         matchables(local_map_->appearances()),
                                         hbst_tree(new HBSTTree(local_map_->identifier(), matchables)) {}
      const LocalMap* local_map;
      const HBSTNode::BinaryMatchableVector matchables;
      const HBSTTree* hbst_tree;
    };

  //ds object management
  public:

    Relocalizer();
    ~Relocalizer();

  //ds functionality
  public:

    //ds initialize relocalization module for a new local map
    void init(const LocalMap* local_map_);

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

    void setPreliminaryMinimumInterspaceQueries(const Count& preliminary_minimum_interspace_queries_) {_preliminary_minimum_interspace_queries = preliminary_minimum_interspace_queries_;}
    void setPreliminaryMinimumMatchingRatio(const real& preliminary_minimum_matching_ratio_) {_preliminary_minimum_matching_ratio = preliminary_minimum_matching_ratio_;}
    void setMinimumNumberOfMatchesPerLandmark(const Count& minimum_number_of_matches_per_landmark_) {_minimum_number_of_matches_per_landmark = minimum_number_of_matches_per_landmark_;}

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

    //ds minimum query interspace
    Count _preliminary_minimum_interspace_queries = 5;

    //ds minimum relative number of matches
    real _preliminary_minimum_matching_ratio = 0.1;

    //ds minimum absolute number of matches
    Count _minimum_number_of_matches_per_landmark = 100;

    //ds correspondence retrieval
    std::set<Identifier> _mask_id_references_for_correspondences;
    Count _minimum_matches_per_correspondence = 0;

    //ds local map to local map alignment
    XYZAligner* _aligner = 0;

    //ds module time consumption
    CREATE_CHRONOMETER(overall)
  };
}
