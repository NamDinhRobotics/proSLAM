#include "frame.h"
#include "local_map.h"
#include "local_map.h"
#include "types/stereo_grid_detector.h"

namespace proslam {
  using namespace std;

  Identifier Frame::_instances = 0;

  Frame::Frame(const TrackingContext* context_,
               Frame* previous_,
               Frame* next_,
               const TransformMatrix3D& robot_to_world_,
               const Identifier& sequence_number_raw_): _index(_instances),
                                                        _sequence_number_raw(sequence_number_raw_),
                                                        _keyframe(0) {
    ++_instances;
    setPrevious(previous_);
    setNext(next_);
    setRobotToWorld(robot_to_world_);
    _points.clear();
    setStatus(Localizing);
  }

  //ds "deep" copy ctor used in keyframe generation - unfortunately error prone as shit currently
  Frame::Frame(Frame* frame_): _index(frame_->index()),
                               _sequence_number_raw(frame_->sequenceNumberRaw()),
                               _keyframe(frame_->keyframe()) {
    setCamera(frame_->camera());
    setPrevious(frame_->previous());
    setNext(frame_->next());
    _status=frame_->status();

    //ds take over point references TODO improve keyframe generation copying
    _points.insert(_points.end(), frame_->points().begin(), frame_->points().end());
    for (FramePoint* frame_point: _points) {
      frame_point->setFrame(this);
    }

    //ds release points from old frame
    frame_->points().clear();

    _robot_to_world          = frame_->robotToWorld();
    _world_to_robot          = frame_->worldToRobot();
    _robot_to_world_odometry = frame_->robotToWorldOdometry();

    _camera                = frame_->camera();
    _camera_extra          = frame_->cameraExtra();
    _intensity_image       = frame_->intensityImage();
    _has_intensity         = frame_->hasIntensity();
    _intensity_image_extra = frame_->intensityImageExtra();
    _has_intensity_extra   = frame_->hasIntensityExtra();
    _has_depth             = frame_->hasDepth();
  }

  Frame::~Frame() {
    for (const FramePoint* frame_point: _points) {
      delete frame_point;
    }
    _points.clear();
  }

  void Frame::setKeyframe(const LocalMap* keyframe_) {
    _keyframe          = keyframe_;
    _frame_to_keyframe = keyframe_->worldToRobot()*this->robotToWorld();
    _keyframe_to_frame = _frame_to_keyframe.inverse();
  }

  bool Frame::isKeyFrame() const {
    if (0 != _keyframe) {
      return (static_cast<const Frame*>(_keyframe) == this);
    } else {
      return false;
    }
  }

  size_t Frame::countPoints(const Count min_age_,
      const ThreeValued has_landmark,
      const ThreeValued has_depth) const{
    size_t count=0;
    for (size_t i=0; i<points().size(); i++){
      const FramePoint* frame_point=points()[i];
      if(frame_point->age()<min_age_) {
	continue;
      }
      if(has_landmark!=Unknown){
	if(has_landmark==True && ! frame_point->landmark())
	  continue;
	if(has_landmark==False && frame_point->landmark())
	  continue;
      }

      if(has_depth!=Unknown){
	if(has_depth==True && ! frame_point->hasDepth())
	  continue;
	if(has_depth==False && frame_point->hasDepth())
	  continue;
      }
      count++;
    }
    return count;
  }

  void Frame::setRobotToWorld(const TransformMatrix3D& robot_to_world_) {
    _robot_to_world = robot_to_world_;
    _world_to_robot = _robot_to_world.inverse();
    updateSubContext();
  }

  FramePoint* Frame::createNewPoint(const cv::KeyPoint& keypoint_left_,
                                    const cv::Mat& descriptor_left_,
                                    const cv::KeyPoint& keypoint_right_,
                                    const cv::Mat& descriptor_right_,
                                    const real& depth_meters_,
                                    const PointCoordinates& coordinates_in_robot_) {

    //ds allocate a new point connected to the previous one
    FramePoint* frame_point = new FramePoint(keypoint_left_,
                                             descriptor_left_,
                                             keypoint_right_,
                                             descriptor_right_,
                                             this);

    //ds this point has no predecessor
    frame_point->setRoot(frame_point);

    //ds update spatials
    frame_point->setRobotCoordinates(coordinates_in_robot_);

    //ds update depth based on quality
    if (depth_meters_ < StereoGridDetector::maximum_depth_close) {
      frame_point->setDepth(depth_meters_);
    } else {
      frame_point->setDepthByVision(depth_meters_);
    }

    //ds update point status
    if (frame_point->age() > minimum_image_age) {
      frame_point->setStatus(FramePoint::Confirmed);
    }
    if (frame_point->age() > minimum_landmark_age) {
      frame_point->setStatus(FramePoint::Persistent);
    }
    return frame_point;
  }

  FramePoint* Frame::createNewPoint(const cv::KeyPoint& keypoint_left_,
                                    const cv::Mat& descriptor_left_,
                                    const cv::KeyPoint& keypoint_right_,
                                    const cv::Mat& descriptor_right_,
                                    const real& depth_meters_,
                                    const PointCoordinates& coordinates_in_robot_,
                                    FramePoint* previous_point_) {

    //ds allocate a new point connected to the previous one
    FramePoint* frame_point = new FramePoint(keypoint_left_,
                                             descriptor_left_,
                                             keypoint_right_,
                                             descriptor_right_,
                                             this,
                                             previous_point_);

    //ds update spatials
    frame_point->setRobotCoordinates(coordinates_in_robot_);

    //ds update depth based on quality
    if (depth_meters_ < StereoGridDetector::maximum_depth_close) {
      frame_point->setDepth(depth_meters_);
    } else {
      frame_point->setDepthByVision(depth_meters_);
    }

    //ds update point status
    if (frame_point->age() > minimum_image_age) {
      frame_point->setStatus(FramePoint::Confirmed);
    }
    if (frame_point->age() > minimum_landmark_age) {
      frame_point->setStatus(FramePoint::Persistent);
    }
    return frame_point;
  }

  Frame* FramePtrMap::get(int index) {
    FramePtrMap::iterator it=find(index);
    if (it==end())
      return 0;
    return it->second;
  }

  void Frame::releaseImages() {
    _intensity_image.release();
    _intensity_image_extra.release();
  }

  void Frame::updateSubContext() {
    //ds empty implementation in base
  }

  void FramePtrMap::put(Frame* frame) {
    FramePtrMap::iterator it=find(frame->index());
    if (it!=end()){
      throw std::runtime_error("FramePtrMap::put(...), double insertion");
    }
    insert(std::make_pair(frame->index(), frame));
  }

  void FramePtrMap::replace(Frame* frame) {
    FramePtrMap::iterator it = find(frame->index());
    if (it!=end()) {
      Frame* frame_to_be_replaced = it->second;

      //ds update parent/child
      frame_to_be_replaced->previous()->setNext(frame);
      frame->setPrevious(frame_to_be_replaced->previous());
      if (0 != frame_to_be_replaced->next()) {
        frame->setNext(frame_to_be_replaced->next());
      }

      //ds free old frame and set new
      delete frame_to_be_replaced;
      it->second = frame;
    } else {
      throw std::runtime_error("cannot replace inexisting frame");
    }
  }
}
