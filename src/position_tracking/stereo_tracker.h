#pragma once
#include "position_tracking/base_tracker.h"
#include "framepoint_generation/stereo_framepoint_generator.h"

namespace proslam {

//ds this class processes two subsequent Frames and establishes Framepoint correspondences (tracks) based on the corresponding images
class StereoTracker: public BaseTracker {

//ds object management
PROSLAM_MAKE_PROCESSING_CLASS(StereoTracker)

//ds functionality
public:

  //ds magic
  virtual void compute();

//ds setters/getters
public:

  void setCameraRight(const Camera* camera_right_) {_camera_right = camera_right_;}
  void setIntensityImageRight(const cv::Mat& intensity_image_right_) {_intensity_image_right = intensity_image_right_;}

//ds helpers
protected:

  //ds creates a frame, which is filled by calling the framepoint generator
  virtual Frame* _createFrame();

  //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  virtual void _recoverPoints(Frame* current_frame_);

//ds attributes
protected:

  //ds configuration
  const Camera* _camera_right = nullptr;

  //ds processing
  cv::Mat _intensity_image_right;

  //ds specified generator instance
  StereoFramePointGenerator* _stereo_framepoint_generator = nullptr;
};
}
