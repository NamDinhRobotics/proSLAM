#pragma once
#include "aligners/aligner_factory.h"
#include "types/local_map.h"

namespace proslam {
  class Relocalizer {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds wrappers
  public:

    //ds TODO remove
    static const HBSTNode::BinaryMatchableVector getMatchables(const AppearancePtrVector& appearances_) {
      assert(appearances_.size() > 0);
      HBSTNode::BinaryMatchableVector matchables(appearances_.size());

      //ds copy raw data
      for (Index index_appearance = 0; index_appearance < appearances_.size(); ++index_appearance) {
        matchables[index_appearance] = new HBSTMatchable(index_appearance, appearances_[index_appearance]->descriptor);
      }
      return matchables;
    }

    //ds TODO remove
    static const AppearancePtrVector getAppearances(const FramePointPtrVector& framepoints_) {
      assert(framepoints_.size() > 0);
      AppearancePtrVector appearances(framepoints_.size());

      //ds copy raw data
      Count number_of_added_appearances = 0;
      for (Index index_appearance = 0; index_appearance < appearances.size(); ++index_appearance) {
        FramePoint* point = framepoints_[index_appearance];
        if (point->landmark() != 0) {
          appearances[number_of_added_appearances] = new Appearance(new LandmarkItem(point->landmark(), point->robotCoordinates()), point->descriptor());
          ++number_of_added_appearances;
        }
      }

      appearances.resize(number_of_added_appearances);
      return appearances;
    }

  //ds exported types
  public:

    struct Query {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      Query(const LocalMap* keyframe_): keyframe(keyframe_),
                                        appearances(keyframe_->appearances()),
                                        matchables(getMatchables(appearances)),
                                        hbst_tree(new HBSTTree(keyframe_->index(), matchables)) {}
      const LocalMap* keyframe = 0;
      const AppearancePtrVector appearances;
      const HBSTNode::BinaryMatchableVector matchables;
      const HBSTTree* hbst_tree = 0;
    };

    struct QueryFrame {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      QueryFrame(const Frame* frame_): frame(frame_),
                                       appearances(getAppearances(frame_->points())),
                                       matchables(getMatchables(appearances)),
                                       hbst_tree(new HBSTTree(frame_->index(), matchables)) {}

      QueryFrame(const LocalMap* keyframe_): frame(keyframe_),
                                             appearances(keyframe_->appearances()),
                                             matchables(getMatchables(appearances)),
                                             hbst_tree(new HBSTTree(keyframe_->index(), matchables)) {}

      ~QueryFrame() {
       for (const Appearance* appearance: appearances) {
         delete appearance->item;
         delete appearance;
       }
       for (const HBSTNode::BinaryMatchable* matchable: matchables) {
         delete matchable;
       }
       delete hbst_tree;
      }

      const Frame* frame = 0;
      const AppearancePtrVector appearances;
      const HBSTNode::BinaryMatchableVector matchables;
      const HBSTTree* hbst_tree = 0;
    };

  //ds object management
  public:

    Relocalizer();
    ~Relocalizer();

  //ds interface
  public:

    //ds initialize closer module for a new keyframe
    void init(const LocalMap* keyframe);

    //ds integrate frame into loop closing pool
    void train();

    //ds flushes all frames in current queue
    void flush();

    //ds retrieve loop closure candidates for the given cloud
    void detect(const bool& force_matching_ = false);

    //ds geometric verification
    void compute();

    //ds retrieve correspondences from matches
    inline const Correspondence* getCorrespondenceNN(const Correspondence::MatchPtrVector& matches_);

    inline const CorrespondenceCollectionPointerVector& closures() const {return _closures;}
    inline void setClosures(CorrespondenceCollectionPointerVector& closures_) {_closures=closures_;}
    void clear();

    //ds configuration
    void setPreliminaryMinimumInterspaceQueries(const Count& preliminary_minimum_interspace_queries_) {_preliminary_minimum_interspace_queries = preliminary_minimum_interspace_queries_;}
    void setPreliminaryMinimumMatchingRatio(const real& preliminary_minimum_matching_ratio_) {_preliminary_minimum_matching_ratio = preliminary_minimum_matching_ratio_;}
    void setMinimumAbsoluteNumberOfMatchesPointwise(const Count& minimum_absolute_number_of_matches_pointwise_) {_minimum_absolute_number_of_matches_pointwise = minimum_absolute_number_of_matches_pointwise_;}

    XYZAligner* aligner() {return _aligner;}

  protected:

    //ds currently found closures
    CorrespondenceCollectionPointerVector _closures;

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
    Count _minimum_absolute_number_of_matches_pointwise = 100;

    //ds correspondence retrieval
    std::set<Identifier> _mask_id_references_for_correspondences;
    Count _minimum_matches_per_correspondence = 0;
    XYZAligner* _aligner = 0;

    //ds module time consumption
    CREATE_CHRONOMETER(overall)

  };
}
