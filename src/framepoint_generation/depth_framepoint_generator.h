#pragma once
#include "base_framepoint_generator.h"

namespace proslam {

//ds this class computes potential framepoints in a stereo image pair by triangulation
class DepthFramePointGenerator: public BaseFramePointGenerator {

//ds object handling
PROSLAM_MAKE_PROCESSING_CLASS(DepthFramePointGenerator)

//ds functionality
public:

  //ds initializes the framepoint generator (e.g. detects keypoints and computes descriptors)
  virtual void initialize(Frame* frame_, const bool& extract_features_ = true);

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  virtual void compute(Frame* frame_);

  void computeCoordinatesFromDepth(Frame* frame_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);

//ds setters/getters
public:

  inline void setCameraRight(const Camera* camera_right_) {_camera_right = camera_right_;}

protected:

  void _computeDepthMap(const cv::Mat& right_depth_image);

//ds settings
protected:

  const Camera* _camera_right;

  const real _maximum_reliable_depth_far_meters = 5;

  //ds inner memory buffers (operated on in compute)
  cv::Mat _space_map_left_meters; // xyz coordinates of every pixel of the left image in meters
  cv::Mat _row_map;               // row index in the depth image(right) corresponding to the pixel ar [r,c] in left image
  cv::Mat _col_map;               // col index in the depth image(right) corresponding to the pixel ar [r,c] in left image

private:

  //ds informative only
  CREATE_CHRONOMETER(depth_map_generation)
  CREATE_CHRONOMETER(depth_assignment)
};
}
