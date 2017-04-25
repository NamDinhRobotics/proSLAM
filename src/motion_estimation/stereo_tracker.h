#pragma once
#include "base_tracker.h"
#include "framepoint_generation/stereo_framepoint_generator.h"

namespace proslam {

  //ds this class processes two subsequent Frames and establishes Framepoint correspondences (tracks) based on the corresponding images
  class StereoTracker: public BaseTracker {

	//ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds the tracker assumes a constant stereo camera configuration
    StereoTracker();

    //ds dynamic cleanup
    ~StereoTracker();

    void setCameraRight(const Camera* camera_right_) { _camera_right=camera_right_;}

    void setIntensityImageRight(const cv::Mat* intensity_image_right_) {_intensity_image_right = intensity_image_right_;}
    virtual void setup();

    virtual void compute();
    
    //ds helpers
  protected:

    //gg
    virtual Frame* _makeFrame();

    //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
    virtual void _recoverPoints(Frame* current_frame_);

  protected:

    // configuration
    const Camera* _camera_right;

    // processing
    const cv::Mat* _intensity_image_right;

    // the one below is a typed copy of the framepoint generator
    // accepts only stereo objects;
    StereoFramePointGenerator* _stereo_framepoint_generator;
  };
}
