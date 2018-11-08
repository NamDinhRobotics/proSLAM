#pragma once
#include "types/definitions.h"



namespace proslam {

//! @struct container holding spatial and appearance information (used in findStereoKeypoints)
struct IntensityFeature {

  IntensityFeature(): row(0), col(0), index_in_vector(0) {}

  IntensityFeature(const cv::KeyPoint& keypoint_,
                         const cv::Mat& descriptor_,
                         const size_t& index_in_vector_): keypoint(keypoint_),
                                                          descriptor(descriptor_),
                                                          row(keypoint_.pt.y),
                                                          col(keypoint_.pt.x),
                                                          index_in_vector(index_in_vector_) {}
  cv::KeyPoint keypoint;    //ds geometric: feature location in 2D
  cv::Mat descriptor;       //ds appearance: feature descriptor
  int32_t row;              //ds pixel column coordinate (v)
  int32_t col;              //ds pixel row coordinate (u)
  size_t index_in_vector; //ds inverted index for vector containing this

};
typedef std::vector<IntensityFeature*> IntensityFeaturePointerVector;

//! @struct support structure
class IntensityFeatureMatcher {
public:

  IntensityFeatureMatcher();
  ~IntensityFeatureMatcher();

public:

  //ds configure matcher
  void setLattice(const int32_t& rows_, const int32_t& cols_);

  //ds create features from keypoints and descriptors
  void setFeatures(const std::vector<cv::KeyPoint>& keypoints_, const cv::Mat& descriptors_);

  //ds sort all input vectors by ascending row positions (preparation for stereo matching)
  void sortFeatureVector();

  //ds performs a local search in a rectangular area on the feature lattice
  IntensityFeature* getMatchingFeatureInRectangularRegion(const int32_t& row_reference_,
                                                                const int32_t& col_reference_,
                                                                const cv::Mat& descriptor_reference_,
                                                                const int32_t& range_point_tracking_pixels_rows_,
                                                                const int32_t& range_point_tracking_pixels_cols_,
                                                                const int32_t& image_rows_,
                                                                const int32_t& image_cols_,
                                                                const real& maximum_descriptor_distance_tracking_);

  //ds prunes features from feature vector if existing
  void prune(const std::set<uint32_t>& matched_indices_);

//ds attributes
public:

  uint32_t number_of_rows = 0;
  uint32_t number_of_cols = 0;
  IntensityFeaturePointerVector feature_vector;
  IntensityFeature*** feature_lattice = nullptr;
  uint32_t number_of_features         = 0;

};
} //namespace proslam
