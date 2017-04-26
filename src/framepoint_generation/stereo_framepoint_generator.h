#pragma once
#include "base_framepoint_generator.h"

namespace proslam {

  //ds this class computes potential framepoints in a stereo image pair by triangulation
  class StereoFramePointGenerator : public BaseFramePointGenerator {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds the stereo camera setup must be provided
    StereoFramePointGenerator();

    //gg to be called after constructor and parameters are set
    virtual void setup();
    
    //ds cleanup of dynamic structures
    virtual ~StereoFramePointGenerator();

  //ds functionality
  public:

    //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
    virtual void compute(Frame* frame_);

    //ds initializes structures for the epipolar stereo keypoint search (called within compute)
    void initialize(const std::vector<cv::KeyPoint>& keypoints_left_,
                    const std::vector<cv::KeyPoint>& keypoints_right_,
                    const cv::Mat& descriptors_left_,
                    const cv::Mat& descriptors_right_);

    //ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
    void findStereoKeypoints(Frame* frame_);

    //ds computes 3D position of a stereo keypoint pair in the keft camera frame (called within findStereoKeypoints)
    const PointCoordinates getCoordinatesInCameraLeft(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const;

  //ds setters/getters
  public:

    inline void setCameraRight(const Camera* camera_right_) {_camera_right = camera_right_;}
    void setMaximumMatchingDistanceTriangulation(const real& maximum_matching_distance_triangulation_) {_maximum_matching_distance_triangulation = maximum_matching_distance_triangulation_;}

  //ds settings
  protected:

    const Camera* _camera_right;

    //ds triangulation properties
    int32_t _maximum_matching_distance_triangulation;
    real _baseline_pixelsmeters;
    real _baseline_meters;
    real _baseline_factor;
    real _minimum_disparity_pixels;

    //ds inner memory buffers (operated on in compute)
    std::vector<cv::KeyPoint> _keypoints_right;
    cv::Mat _descriptors_right;
    std::vector<KeypointWithDescriptor> _keypoints_with_descriptors_right;

    //ds informative only
    CREATE_CHRONOMETER(point_triangulation)
  };
}
