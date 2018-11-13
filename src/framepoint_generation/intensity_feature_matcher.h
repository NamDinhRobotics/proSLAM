#pragma once
#include "intensity_feature_extractor.h"
#include "types/frame_point.h"



namespace proslam {

//! @struct support structure
class IntensityFeatureMatcher {
public:

  IntensityFeatureMatcher();
  ~IntensityFeatureMatcher();

public:

  //ds configure matcher
  void configure(const int32_t& rows_, const int32_t& cols_);

  //ds create features from keypoints and descriptors
  void setFeatures(const std::vector<cv::KeyPoint>& keypoints_, const cv::Mat& descriptors_);

  //ds sort all input vectors by ascending row positions (preparation for stereo matching)
  void sortFeatureVector();

  //ds performs a local search in a rectangular area on the feature lattice
  IntensityFeature* getMatchingFeatureInRectangularRegion(const int32_t& row_reference_,
                                                          const int32_t& col_reference_,
                                                          const cv::Mat& descriptor_reference_,
                                                          const int32_t& row_start_point,
                                                          const int32_t& row_end_point,
                                                          const int32_t& col_start_point,
                                                          const int32_t& col_end_point,
                                                          const real& maximum_descriptor_distance_tracking_,
                                                          const bool track_by_appearance_,
                                                          real& descriptor_distance_best_);

  //ds prunes features from feature vector if existing
  void prune(const std::set<uint32_t>& matched_indices_);

//ds attributes
public:

  int32_t number_of_rows = 0;
  int32_t number_of_cols = 0;
  IntensityFeaturePointerVector feature_vector;
  IntensityFeature*** feature_lattice = nullptr;

};
} //namespace proslam
