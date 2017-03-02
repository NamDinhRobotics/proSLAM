#pragma once
#include <exception>

#include "contexts/gt_keyframe.h"
#include "utilities/gt_utility.h"

namespace gslam {

  struct Match {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Match(const LandmarkItem* item_query_,
          const LandmarkItem* item_reference_,
          const Count& matching_distance_hamming_): item_query(item_query_),
                                                    item_reference(item_reference_),
                                                    matching_distance_hamming(matching_distance_hamming_) {}

    Match(const Match* match_): item_query(match_->item_query),
                                item_reference(match_->item_reference),
                                matching_distance_hamming(match_->matching_distance_hamming) {}

    const LandmarkItem* item_query        = 0;
    const LandmarkItem* item_reference    = 0;
    const Count matching_distance_hamming = 0;
  };
  typedef std::vector<const Match*> MatchPtrVector;
  typedef std::map<const Identifier, MatchPtrVector> MatchMap;
  typedef std::pair<const Identifier, MatchPtrVector> MatchMapElement;

  struct Query {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Query(const KeyFrame* keyframe_): keyframe(keyframe_),
                                      appearances(keyframe_->appearances()),
                                      matchables(Utility::getMatchables(appearances)),
                                      //descriptors_bow(Utility::getDescriptorsBoW(appearances)),
                                      hbst_tree(new HBSTTree(keyframe_->index(), matchables)) {}
    const KeyFrame* keyframe = 0;
    const AppearancePtrVector appearances;
    const HBSTNode::BinaryMatchableVector matchables;
    //const std::vector<DBoW2::FBrief::TDescriptor> descriptors_bow;
    const HBSTTree* hbst_tree = 0;
  };
  //typedef std::map<const DBoW2::EntryId, const Query*> QueryMap;
  //typedef std::pair<const DBoW2::EntryId, const Query*> QueryMapElement;

  struct QueryFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    QueryFrame(const Frame* frame_): frame(frame_),
                                     appearances(Utility::getAppearances(frame_->points())),
                                     matchables(Utility::getMatchables(appearances)),
                                     hbst_tree(new HBSTTree(frame_->index(), matchables)) {}

    QueryFrame(const KeyFrame* keyframe_): frame(keyframe_),
                                           appearances(keyframe_->appearances()),
                                           matchables(Utility::getMatchables(appearances)),
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

  //ds sorry boss :>
  class ExceptionNoTransformFound: public std::exception {
  public:

    ExceptionNoTransformFound(const std::string& what_): _what(what_){}
    ~ExceptionNoTransformFound(){}

  public:

    virtual const char* what() const throw() {return _what.c_str();}

  private:

    const std::string _what;
  };
} //namespace gtracker
