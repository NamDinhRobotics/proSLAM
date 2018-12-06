#pragma once
#include "base_framepoint_generator.h"

namespace proslam {

//ds this class computes potential framepoints in a stereo image pair by triangulation
class StereoFramePointGenerator : public BaseFramePointGenerator {

//ds object handling
PROSLAM_MAKE_PROCESSING_CLASS(StereoFramePointGenerator)

//ds functionality
public:

  //ds initializes the framepoint generator (e.g. detects keypoints and computes descriptors in the left and right camera image)
  virtual void initialize(Frame* frame_, const bool& extract_features_ = true);

  //! @brief computes framepoints based on exhaustive, rigid stereo matching on multiple epipolar lines without prior
  //! @param[in, out] frame_ frame that will be filled with framepoints
  virtual void compute(Frame* frame_);

  //! @brief computes first tracks between previous framepoints, estimates the projection error
  //! @brief and uses the prior on the right camera for fast and reliable stereo matching
  //! @param[in, out] frame_ frame that will be filled with framepoints and tracks
  //! @param[in] frame_previous_ previous frame that contains valid framepoints on which we will track
  //! @param[in] camera_left_previous_in_current_ the relative camera motion guess between frame_ and frame_previous_
  //! @param[out] previous_points_without_tracks_ lost points
  void track(Frame* frame_,
             Frame* frame_previous_,
             const TransformMatrix3D& camera_left_previous_in_current_,
             FramePointPointerVector& previous_framepoints_without_tracks_,
             const bool track_by_appearance_ = true) override;

  //ds computes 3D position of a stereo keypoint pair in the keft camera frame
  const PointCoordinates getPointInLeftCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const;

//ds setters/getters
public:

  inline void setCameraRight(const Camera* camera_right_) {_camera_right = camera_right_;}
  const real& meanTriangulationSuccessRatio() const {return _mean_triangulation_success_ratio;}

//ds settings
protected:

  //! @brief right camera handle
  const Camera* _camera_right = 0;

  //! @brief derived triangulation properties - updated to current camera configuration before each compute
  Vector3 _baseline           = Vector3::Zero();
  real _baseline_pixelsmeters = 0;
  real _baseline_meters       = 0;
  real _f_x = 0;
  real _f_y = 0;
  real _c_x = 0;
  real _c_y = 0;
  real _b_x = 0;

  //! @brief current epipolar search range
  int32_t _maximum_epipolar_search_offset_pixels = 0;

  //! @brief current triangulation distance
  real _current_maximum_descriptor_distance_triangulation = 0.1*SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS;

  //! @brief information only: average triangulation success ratio
  real _mean_triangulation_success_ratio = 1;
  Count _number_of_triangulations = 1;

  //! @brief horizontal epipolar stereo matching search offsets (to consider for stereo matching)
  std::vector<int32_t> _epipolar_search_offsets_pixel;

  //! @brief feature matching class (maintains features in a 2D lattice corresponding to the image and a vector)
  IntensityFeatureMatcher _feature_matcher_right;

private:

  //ds informative only
  CREATE_CHRONOMETER(point_triangulation)
};
}
