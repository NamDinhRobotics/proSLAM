#pragma once
#include "position_tracking/base_tracker.h"
#include "framepoint_generation/depth_framepoint_generator.h"

namespace proslam {

//ds this class processes two subsequent Frames and establishes Framepoint correspondences (tracks) based on the corresponding images
class DepthTracker: public BaseTracker {

//ds object management
PROSLAM_MAKE_PROCESSING_CLASS(DepthTracker)

//ds functionality
public:

  virtual void compute();

//ds setters/getters
public:

  void setDepthCamera(const Camera* depth_camera_) {_depth_camera = depth_camera_;}
  void setDepthImage(const cv::Mat& depth_image_) {_depth_image = depth_image_;}

//ds helpers
protected:

  //gg
  virtual Frame* _createFrame();

  //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  virtual void _recoverPoints(Frame* current_frame_);

//ds attributes
protected:

  //ds configuration
  const Camera* _depth_camera = nullptr;

  //ds processing
  cv::Mat _depth_image;

  //ds specified generator instance
  DepthFramePointGenerator* _depth_framepoint_generator = nullptr;
};
}
