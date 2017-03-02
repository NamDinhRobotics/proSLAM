#pragma once
#include "types/items/correspondence.h"
#include "types/gt_relocalizer_types.h"

namespace gslam {

  class CorrespondenceCollection: public BaseContext {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  //ds object handling
  public:

    //ds ctor
    CorrespondenceCollection(const KeyFrame* keyframe_query_,
                             const KeyFrame* keyframe_reference_,
                             const Count& absolute_number_of_matches_,
                             const gt_real& relative_number_of_matches_,
                             const MatchMap& matches_,
                             const CorrespondencePointerVector& correspondences_);

    //ds deep copy ctor
    CorrespondenceCollection(CorrespondenceCollection* collection_);

    ~CorrespondenceCollection();

    //ds deep context merging (old element can be deleted)
    void absorb(const CorrespondenceCollection* collection_);

    //ds deep copy helpers
    static const MatchMap getClone(const MatchMap& matches_per_point_) {
      MatchMap matches_per_point_copy;
      for (const MatchMapElement matches_element: matches_per_point_) {
        MatchPtrVector matches_copy(matches_element.second.size());
        for(Index index_match = 0; index_match < matches_element.second.size(); ++index_match) {
          matches_copy[index_match] = new Match(matches_element.second[index_match]);
        }
        matches_per_point_copy.insert(std::make_pair(matches_element.first, matches_copy));
      }
      return matches_per_point_copy;
    }

    static const CorrespondencePointerVector getClone(const CorrespondencePointerVector& correspondences_) {
      CorrespondencePointerVector correspondences_copy(correspondences_.size());
      for(Index index_correspondence = 0; index_correspondence < correspondences_.size(); ++index_correspondence) {
        correspondences_copy[index_correspondence] = new Correspondence(correspondences_[index_correspondence]);
      }
      return correspondences_copy;
    }

    const KeyFrame* keyframe_query     = 0;
    const KeyFrame* keyframe_reference = 0;
    const Identifier id_query          = 0;
    const Identifier id_reference      = 0;
    const Count absolute_number_of_matches   = 0;
    const gt_real relative_number_of_matches = 0.0;
    const MatchMap matches_per_point;
    CorrespondencePointerVector correspondences;
    TransformMatrix3D transform_frame_query_to_frame_reference = TransformMatrix3D::Identity();
    gt_real icp_inlier_ratio       = 0.0;
    Count icp_number_of_iterations = 0;
    Count icp_number_of_inliers    = 0;
    bool is_valid              = false;
  };

  typedef std::vector<CorrespondenceCollection*> CorrespondenceCollectionPointerVector;
  typedef std::map<const Identifier, CorrespondenceCollection*> CorrespondenceMap;
  typedef std::pair<const Identifier, CorrespondenceCollection*> CorrespondenceMapElement;

  //ds architecture CorrespondenceMultiMap: once a query-reference pair is merged, the query is removed from the map
  //map-> query1-> reference2 + correspondences1-2
  // |       |---> reference5 + correspondences1-5
  // |
  // ---> query2-> reference5 + correspondences2-5
  //         |---> reference3 + correspondences2-3
  //         |---> reference4 + correspondences2-4
  //...
  typedef std::map<const Identifier, std::map<const Identifier, CorrespondenceCollection*>> CorrespondenceMultiMap;
  typedef std::pair<const Identifier, std::map<const Identifier, CorrespondenceCollection*>> CorrespondenceMultiMapElement;
} //namespace gslam
