#pragma once
#include "../relocalizer_types.h"

namespace proslam{
  class TrackingContext{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TrackingContext();
    ~TrackingContext();

    const TransformMatrix3D& currentToPreviousContext() const {return _current_to_previous_initial;}
    const TransformMatrix3D& previousToCurrentContext() const {return _previous_to_current_initial;}

    void clear();

    FramePtrMap& frames() {return _frames;}
    KeyFramePtrVector& keyframes() {return _keyframes;}
    const KeyFramePtrVector& keyframes() const {return _keyframes;}
    Frame* createNewFrame(const TransformMatrix3D& robot_pose, const Identifier& sequence_number_raw_ = 0);
    const bool createLocalMap();
    const FramePtrVector& frameQueueForKeyframe() const {return _frame_queue_for_keyframe;}

    LandmarkPtrMap& landmarks() {return _landmarks;}
    Landmark* createNewLandmark(const PointCoordinates& coordinates_in_world_ = PointCoordinates::Zero());

    Frame* rootFrame() {return _root_frame;}
    const Frame* currentFrame() const {return _current_frame;}
    const Frame* previousFrame() const {return _previous_frame;}
    Frame* currentFrame() {return _current_frame;}
    Frame* previousFrame() {return _previous_frame;}
    KeyFrame* currentKeyframe() {return _current_keyframe;}
    KeyFrame* previousKeyframe() {assert(1 < _keyframes.size()); return *(_keyframes.end()-2);} //ds NASTY, price for readability
    void closeKeyframes(KeyFrame* query_, const KeyFrame* reference_, const TransformMatrix3D& transform_query_to_reference_);

    cv::DescriptorExtractor* descriptorExtractor() {return _descriptor_extractor;}

    void setRobotToWorldPrevious(const TransformMatrix3D& robot_pose_) {_last_good_robot_pose = robot_pose_;}
    const TransformMatrix3D robotToWorldPrevious() const {return _last_good_robot_pose;}

    const bool closedKeyframe() const {return _closed_keyframe;}

    void resetWindow();

    void purifyLandmarks();

    //ds dump trajectory to file (in KITTI benchmark format only for now)
    void writeTrajectory(const std::string& filename_ = "") const;

  protected:

    const TransformMatrix3D _current_to_previous_initial;
    const TransformMatrix3D _previous_to_current_initial;
    Frame* _root_frame     = 0;
    Frame* _current_frame  = 0;
    Frame* _previous_frame = 0;
    LandmarkPtrMap _landmarks;
    FramePtrMap _frames;

#if CV_MAJOR_VERSION == 2
    cv::DescriptorExtractor* _descriptor_extractor = 0;
#elif CV_MAJOR_VERSION == 3
    const cv::Ptr<cv::Feature2D> _descriptor_extractor;
#else
  #error OpenCV version not supported
#endif

    TransformMatrix3D _last_good_robot_pose  = TransformMatrix3D::Identity();
    bool _closed_keyframe    = false;

    //ds current frame window buffer for key frame generation
    real _distance_traveled_window = 0.0;
    real _degrees_rotated_window   = 0.0;

    //ds key frame generation properties
    real _minimum_distance_traveled_for_keyframe = 0.5; //ds keyframe generation based on translational movement
    real _minimum_degrees_rotated_for_keyframe   = 0.5; //ds keyframe generation based on rotational movement
    Count _minimum_number_of_frames_for_keyframe    = 4;   //ds in case translational keyframe generation is triggered, this value enforces a reasonable trajectory granularity
    
    FramePtrVector _frame_queue_for_keyframe;

    //ds key frame holders
    KeyFrame* _current_keyframe  = 0;
    KeyFramePtrVector _keyframes;
  };

  typedef std::vector<TrackingContext*> TrackingContextPointerVector;
  typedef std::map<Identifier, TrackingContext*> TrackingContextPointerMap;
  typedef std::pair<Identifier, TrackingContext*> TrackingContextPointerMapElement;
}
