#include "depth_tracker.h"

namespace proslam {
  using namespace srrg_core;

  //ds the tracker assumes a constant stereo camera configuration
  DepthTracker::DepthTracker(){
    _camera_right=0;
    _depth_image_right=0;
    _depth_framepoint_generator=0;
    std::cerr << "DepthTracker::DepthTracker|constructed" << std::endl;
  }

  void DepthTracker::setup() {
    BaseTracker::setup();
    assert(_camera_right);
    _depth_framepoint_generator = dynamic_cast<DepthFramePointGenerator*>(_framepoint_generator);
    assert(_depth_framepoint_generator);
  }

  //ds dynamic cleanup
  DepthTracker::~DepthTracker() {
    std::cerr << "DepthTracker::DepthTracker|destroying" << std::endl;
    std::cerr << "DepthTracker::DepthTracker|destroyed" << std::endl;
  }

  Frame* DepthTracker::_makeFrame(){
    Frame* current_frame = _context->createFrame(_context->robotToWorld(), _framepoint_generator->maximumDepthNearMeters());
    current_frame->setCameraLeft(_camera_left);
    current_frame->setIntensityImageLeft(*_intensity_image_left);
    current_frame->setCameraRight(_camera_right);
    current_frame->setIntensityImageRight(*_depth_image_right);
    return current_frame;
  }
  
  //ds creates a new Frame for the given images, retrieves the correspondences relative to the previous Frame, optimizes the current frame pose and updates landmarks
  void DepthTracker::compute() {
    assert(_depth_image_right);
    BaseTracker::compute();
  }

  //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  void DepthTracker::_recoverPoints(Frame* current_frame_) {
    return;
  }
}
