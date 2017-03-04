#pragma once
#include "../camera.h"
#include "../items/frame_point.h"

namespace gslam {
  
  class KeyFrame;
  class TrackingContext;
  class Frame: public BaseContext {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:

    enum Status {Localizing, Tracking}; //< defines one of the two tracker states
    Frame(const TrackingContext* context_,
        Frame* previous_,
        Frame* next_,
        const TransformMatrix3D& robot_to_world_,
        const Identifier& sequence_number_raw_ = 0);
    Frame(Frame* frame_);
    virtual ~Frame();
    Frame() = delete;

  public:

    const Identifier& index() const {return _index;}
    const Identifier& sequenceNumberRaw() const {return _sequence_number_raw;}

    inline Frame* previous() {return _previous;}
    inline const Frame* previous() const {return _previous;}
    inline void setPrevious(Frame* previous_) {_previous=previous_;}

    inline Frame* next()  {return _next;}
    inline const Frame* next() const {return _next;}
    inline void setNext(Frame* next_) {_next=next_;}

    inline const Camera* camera() const {return _camera;}
    inline void setCamera(const Camera* camera_) {_camera = camera_;}

    inline const Camera* cameraExtra() const {return _camera_extra;}
    inline void setCameraExtra(const Camera* camera_extra_) {_camera_extra = camera_extra_;}

    inline const TransformMatrix3D& robotToWorld() const {return _robot_to_world;}
    void setRobotToWorld(const TransformMatrix3D& robot_to_world_);
    inline const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}
    
    inline const TransformMatrix3D& frameToKeyframe() const {return _frame_to_keyframe;}
    inline const TransformMatrix3D& keyframeToFrame() const {return _keyframe_to_frame;}

    //ds visualization only
    void setRobotToWorldOdometry(const TransformMatrix3D& robot_to_world_) {_robot_to_world_odometry = robot_to_world_;}
    const TransformMatrix3D& robotToWorldOdometry() const {return _robot_to_world_odometry;}

    inline const FramePointPtrVector& points() const {return _points;}
    inline FramePointPtrVector& points() {return _points;}

    //ds mono point factory method 1
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_);

    //ds mono factory method 2
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_,
                               FramePoint* previous_point_);

    //ds stereo point factory method 1
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_,
                               const cv::KeyPoint& keypoint_right_,
                               const cv::Mat& descriptor_right_,
                               const gt_real& depth_meters_,
                               const PointCoordinates& coordinates_in_robot_);

    //ds stereo factory method 2
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_,
                               const cv::KeyPoint& keypoint_right_,
                               const cv::Mat& descriptor_right_,
                               const gt_real& depth_meters_,
                               const PointCoordinates& coordinates_in_robot_,
                               FramePoint* previous_point_);

    inline const IntensityImage& intensityImage() const {return _intensity_image;}
    inline void  setIntensityImage(const IntensityImage& intensity_image_)  {_intensity_image = intensity_image_.clone(); _has_intensity = true;}
    inline const bool hasIntensity() const {return _has_intensity;}

    inline const IntensityImage& intensityImageExtra() const {return _intensity_image_extra;}
    inline void  setIntensityImageExtra(const IntensityImage& intensity_image_extra_)  {_intensity_image_extra = intensity_image_extra_.clone(); _has_intensity_extra = true;}
    inline const bool hasIntensityExtra() const {return _has_intensity_extra;}

    inline const DepthImage& depthImage() const {return _depth_image;}
    inline void  setDepthImage(const DepthImage& depth_image) {_depth_image = depth_image.clone(); _has_depth = true;}
    inline const bool hasDepth() const {return _has_depth;}

    inline Status status() const {return _status;}
    inline void setStatus(Status status_) {_status=status_;}

    void setKeyframe(const KeyFrame* keyframe);
    const KeyFrame* keyframe() const {return _keyframe;}
    bool isKeyFrame() const;

    void setTrackingContext(const TrackingContext* tracking_context_) {_tracking_context = tracking_context_; }
    const TrackingContext* trackingContext() const {return _tracking_context;}

    void toCvPoints(std::vector<cv::Point2f>& cv_points) const;
    void toCvPointsExtra(std::vector<cv::Point2f>& cv_points) const;

    void toCvKeyPoints(std::vector<cv::KeyPoint>& cv_keypoints) const;

    size_t countPoints(const Count min_age_,
		       const ThreeValued has_landmark = Unknown,
		       const ThreeValued has_depth = Unknown) const ;

    void releaseImages();

    //ds subcontext update: propagate changes to subcontext elements (e.g. frames in a keyframe)
    virtual void updateSubContext();

  public:

    //ds singleton configuration
    static constexpr Count minimum_landmark_age = 3;
    static constexpr Count minimum_image_age    = 1;

  protected:

    const Identifier _index;
    const Identifier _sequence_number_raw;
    Status _status;
    Frame* _previous = 0;
    Frame* _next     = 0;

    //ds frame point generation
    FramePointPtrVector _points;

    //ds spatials
    TransformMatrix3D _frame_to_keyframe       = TransformMatrix3D::Identity();
    TransformMatrix3D _keyframe_to_frame       = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world          = TransformMatrix3D::Identity();
    TransformMatrix3D _world_to_robot          = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world_odometry = TransformMatrix3D::Identity();

    //ds TODO refactor to support arbitrary camera constellations -> check nicp
    const Camera* _camera        = 0;
    const Camera* _camera_extra  = 0;

    //ds TODO refactor to support arbitrary number of rgb/depth image combinations
    IntensityImage _intensity_image;
    bool _has_intensity                      = false;
    IntensityImage _intensity_image_extra;
    bool _has_intensity_extra                = false;
    DepthImage _depth_image;
    bool _has_depth                          = false;

    const KeyFrame* _keyframe                = 0;
    const TrackingContext* _tracking_context = 0;
    static Identifier _instances;

  friend class Tracker;

  };

  typedef std::vector<Frame*> FramePtrVector;
  typedef std::pair<int, Frame*> FramePtrMapElement;

  class FramePtrMap: public std::map<int, Frame*>{
  public:

    Frame* get(int index);
    void put(Frame* frame);
    void replace(Frame* frame);

  };
} //namespace gtracker
