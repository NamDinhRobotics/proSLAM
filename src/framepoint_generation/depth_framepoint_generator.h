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
  virtual void initialize(Frame* frame_, const bool& extract_features_ = true) override;

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  virtual void compute(Frame* frame_) override;

  //! @brief computes first tracks between previous framepoints, estimates the projection error
  //! @brief and uses the prior on the right camera for fast and reliable stereo matching
  //! @param[in, out] frame_ frame that will be filled with framepoints and tracks
  //! @param[in] frame_previous_ previous frame that contains valid framepoints on which we will track
  //! @param[in] camera_left_previous_in_current_ the relative camera motion guess between frame_ and frame_previous_
  //! @param[out] previous_points_without_tracks_ lost points
  virtual void track(Frame* frame_,
                     Frame* frame_previous_,
                     const TransformMatrix3D& camera_left_previous_in_current_,
                     FramePointPointerVector& previous_framepoints_without_tracks_,
                     const bool track_by_appearance_ = true) override;

  //! @brief attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  //! @brief param[in] current_frame_ the affected frame carrying points to be recovered
  virtual void recoverPoints(Frame* current_frame_, const FramePointPointerVector& lost_points_) const override;

//ds setters/getters
public:

  inline void setCameraRight(const Camera* camera_right_) {_camera_right = camera_right_;}

protected:

  void _computeDepthMap(cv::Mat& right_depth_image);

//ds settings
protected:

  const Camera* _camera_right = nullptr;

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
