#pragma once
#include "base_framepoint_generator.h"

namespace proslam {

  //ds this class computes potential framepoints in a stereo image pair by triangulation
  class DepthFramePointGenerator : public BaseFramePointGenerator {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds object handling
  public:

    //ds the stereo camera setup must be provided
    DepthFramePointGenerator();

    inline void setCameraRight(const Camera* camera_right_) {_camera_right=camera_right_;}
    
    virtual void setup();
    
    //ds cleanup of dynamic structures
    virtual ~DepthFramePointGenerator();

  //ds functionality
  public:

    //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
    virtual void compute(Frame* frame_);

    void computeCoordinatesFromDepth(Frame* frame_);

  protected:

    void _computeDepthMap(const cv::Mat& right_depth_image);

  //ds settings
  protected:
    const Camera* _camera_right;


    //ds inner memory buffers (operated on in compute)
    cv::Mat _space_map_left_meters; // xyz coordinates of every pixel of the left image in meters

    cv::Mat _row_map;               // row index in the depth image(right) corresponding to the pixel ar [r,c] in left image
    cv::Mat _col_map;               // col index in the depth image(right) corresponding to the pixel ar [r,c] in left image

    //ds informative only
    CREATE_CHRONOMETER(depth_map_generation)
    CREATE_CHRONOMETER(depth_assignment)
  };
}
