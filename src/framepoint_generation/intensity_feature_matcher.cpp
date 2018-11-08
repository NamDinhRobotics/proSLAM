#include "intensity_feature_matcher.h"
#include "types/definitions.h"



namespace proslam {

IntensityFeatureMatcher::IntensityFeatureMatcher() {feature_vector.clear();}

IntensityFeatureMatcher::~IntensityFeatureMatcher() {
  for (uint32_t r = 0; r < number_of_rows; ++r) {
    delete[] feature_lattice[r];
  }
  delete[] feature_lattice;
  feature_vector.clear();
}

void IntensityFeatureMatcher::setLattice(const int32_t& rows_, const int32_t& cols_) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::runtime_error("KeypointWithDescriptorLattice::setStereoMatchCandidateGrid|invalid image dimensions");
  }
  if (feature_lattice) {
    throw std::runtime_error("KeypointWithDescriptorLattice::setStereoMatchCandidateGrid|lattice already allocated");
  }

  //ds initialize empty lattice
  feature_lattice = new IntensityFeature**[rows_];
  for (int32_t r = 0; r < rows_; ++r) {
    feature_lattice[r] = new IntensityFeature*[cols_];
    for (int32_t c = 0; c < cols_; ++c) {
      feature_lattice[r][c] = nullptr;
    }
  }
  number_of_rows = rows_;
  number_of_cols = cols_;
}

void IntensityFeatureMatcher::setFeatures(const std::vector<cv::KeyPoint>& keypoints_, const cv::Mat& descriptors_) {
  if (keypoints_.size() != static_cast<size_t>(descriptors_.rows)) {
    throw std::runtime_error("KeypointWithDescriptorLattice::setFeatures|mismatching keypoints and descriptor numbers");
  }

  //ds clear the lattice - freeing remaining features
  for (uint32_t r = 0; r < number_of_rows; ++r) {
    for (uint32_t c = 0; c < number_of_cols; ++c) {
      feature_lattice[r][c] = nullptr;
    }
  }
  for (IntensityFeature* feature: feature_vector) {
    delete feature;
  }

  //ds fill in features
  number_of_features = 0;
  feature_vector.resize(keypoints_.size());
  for (uint32_t index = 0; index < keypoints_.size(); ++index) {
    IntensityFeature* feature = new IntensityFeature(keypoints_[index], descriptors_.row(index), index);
    feature_vector[index] = feature;
    feature_lattice[feature->row][feature->col] = feature;
    ++number_of_features;
  }
}

void IntensityFeatureMatcher::sortFeatureVector() {
  std::sort(feature_vector.begin(), feature_vector.end(),  [](const IntensityFeature* a_, const IntensityFeature* b_){
    return ((a_->row < b_->row) || (a_->row == b_->row && a_->col < b_->col));
  });
}

IntensityFeature* IntensityFeatureMatcher::getMatchingFeatureInRectangularRegion(const int32_t& row_reference_,
                                                              const int32_t& col_reference_,
                                                              const cv::Mat& descriptor_reference_,
                                                              const int32_t& range_point_tracking_pixels_rows_,
                                                              const int32_t& range_point_tracking_pixels_cols_,
                                                              const int32_t& image_rows_,
                                                              const int32_t& image_cols_,
                                                              const real& maximum_descriptor_distance_tracking_) {
  real descriptor_distance_best = maximum_descriptor_distance_tracking_;
  int32_t row_best = -1;
  int32_t col_best = -1;

  //ds current tracking region
  const int32_t row_start_point = std::max(row_reference_-range_point_tracking_pixels_rows_, 0);
  const int32_t row_end_point   = std::min(row_reference_+range_point_tracking_pixels_rows_+1, image_rows_);
  const int32_t col_start_point = std::max(col_reference_-range_point_tracking_pixels_cols_, 0);
  const int32_t col_end_point   = std::min(col_reference_+range_point_tracking_pixels_cols_+1, image_cols_);

  //ds locate best match in appearance
  for (int32_t row = row_start_point; row < row_end_point; ++row) {
    for (int32_t col = col_start_point; col < col_end_point; ++col) {
      if (feature_lattice[row][col]) {
        const real descriptor_distance = cv::norm(descriptor_reference_,
                                                  feature_lattice[row][col]->descriptor,
                                                  SRRG_PROSLAM_DESCRIPTOR_NORM);

        if (descriptor_distance < descriptor_distance_best) {
          descriptor_distance_best = descriptor_distance;
          row_best = row;
          col_best = col;
        }
      }
    }
  }

  //ds if we found a match
  if (row_best != -1) {
    return feature_lattice[row_best][col_best];
  } else {
    return nullptr;
  }
}

void IntensityFeatureMatcher::prune(const std::set<uint32_t>& matched_indices_) {

  //ds remove matched indices from candidate pools
  uint32_t number_of_unmatched_elements = 0;
  for (uint32_t index = 0; index < feature_vector.size(); ++index) {

    //ds if we haven't matched this index yet
    if (matched_indices_.count(index) == 0) {

      //ds keep the element (this operation is not problemenatic since we do not loop reversely here)
      feature_vector[number_of_unmatched_elements] = feature_vector[index];
      ++number_of_unmatched_elements;
    }
  }
  feature_vector.resize(number_of_unmatched_elements);
}
} //namespace proslam
