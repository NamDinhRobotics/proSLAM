#include "depth_tracker.h"

namespace proslam {
  using namespace srrg_core;

  DepthTracker::DepthTracker(DepthTrackerParameters* parameters_): BaseTracker(parameters_),
                                                                   _parameters(parameters_) {
    LOG_INFO(std::cerr << "DepthTracker::DepthTracker|constructed" << std::endl)
  }

  void DepthTracker::configure() {
    LOG_INFO(std::cerr << "DepthTracker::configure|configuring" << std::endl)
    BaseTracker::configure();
    assert(_depth_camera);
    _depth_framepoint_generator = dynamic_cast<DepthFramePointGenerator*>(_framepoint_generator);
    assert(_depth_framepoint_generator);
    LOG_INFO(std::cerr << "DepthTracker::configure|configured" << std::endl)
  }

  DepthTracker::~DepthTracker() {
    LOG_INFO(std::cerr << "DepthTracker::~DepthTracker|destroyed" << std::endl)
  }

  Frame* DepthTracker::_createFrame(){
    Frame* current_frame = _context->createFrame(_context->robotToWorld());
    current_frame->setCameraLeft(_camera_left);
    current_frame->setIntensityImageLeft(_intensity_image_left);
    current_frame->setCameraRight(_depth_camera);
    current_frame->setIntensityImageRight(_depth_image);
    return current_frame;
  }
  
  void DepthTracker::compute() {
    BaseTracker::compute();
  }

  void DepthTracker::_recoverPoints(Frame* current_frame_) {
    return;
  }
}
