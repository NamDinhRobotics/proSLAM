#pragma once
#include "base_context.h"
#include "camera.h"
#include "frame_point.h"

namespace proslam {
  
  class LocalMap;
  class WorldMap;
  class Frame: public BaseContext {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:

    //ds defines one of the two tracker states
    enum Status {Localizing, Tracking};
    Frame(const WorldMap* context_,
          Frame* previous_,
          Frame* next_,
          const TransformMatrix3D& robot_to_world_,
          const real& maximum_depth_close_);
    Frame(Frame* frame_);
    virtual ~Frame();
    Frame() = delete;

  public:

    const Identifier& index() const {return _index;}

    inline Frame* previous() {return _previous;}
    inline const Frame* previous() const {return _previous;}
    inline void setPrevious(Frame* previous_) {_previous = previous_;}

    inline Frame* next()  {return _next;}
    inline const Frame* next() const {return _next;}
    inline void setNext(Frame* next_) {_next = next_;}

    inline const Camera* cameraLeft() const {return _camera_left;}
    inline void setCameraLeft(const Camera* camera_) {_camera_left = camera_;}

    inline const Camera* cameraRight() const {return _camera_right;}
    inline void setCameraRight(const Camera* camera_) {_camera_right = camera_;}

    inline const TransformMatrix3D& robotToWorld() const {return _robot_to_world;}
    void setRobotToWorld(const TransformMatrix3D& robot_to_world_);
    inline const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}
    
    inline const TransformMatrix3D& frameToLocalMap() const {return _frame_to_local_map;}
    inline const TransformMatrix3D& localMapToFrame() const {return _local_map_to_frame;}

    //ds visualization only
    void setRobotToWorldOdometry(const TransformMatrix3D& robot_to_world_) {_robot_to_world_odometry = robot_to_world_;}
    const TransformMatrix3D& robotToWorldOdometry() const {return _robot_to_world_odometry;}

    inline const FramePointPtrVector& points() const {return _points;}
    inline FramePointPtrVector& points() {return _points;}

    //ds stereo point factory method 1
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_,
                               const cv::KeyPoint& keypoint_right_,
                               const cv::Mat& descriptor_right_,
                               const real& depth_meters_,
                               const PointCoordinates& coordinates_in_robot_);

    //ds stereo factory method 2
    FramePoint* createNewPoint(const cv::KeyPoint& keypoint_left_,
                               const cv::Mat& descriptor_left_,
                               const cv::KeyPoint& keypoint_right_,
                               const cv::Mat& descriptor_right_,
                               const real& depth_meters_,
                               const PointCoordinates& coordinates_in_robot_,
                               FramePoint* previous_point_);

    inline const IntensityImage& intensityImageLeft() const {return _intensity_image_left;}
    inline void setIntensityImageLeft(const IntensityImage& intensity_image_)  {_intensity_image_left = intensity_image_.clone();}

    inline const IntensityImage& intensityImageRight() const {return _intensity_image_right;}
    inline void setIntensityImageRight(const IntensityImage& intensity_image_)  {_intensity_image_right = intensity_image_.clone();}

    inline Status status() const {return _status;}
    inline void setStatus(const Status& status_) {_status = status_;}

    inline const real maximumDepthClose() const {return _maximum_depth_close;}

    void setLocalMap(const LocalMap* local_map_);
    const LocalMap* localMap() const {return _local_map;}
    bool isLocalMapAnchor() const;

    const Count countPoints(const Count min_age_,
		                        const ThreeValued has_landmark_ = Unknown) const;

    void releaseImages();

    //ds subcontext update: propagate changes to subcontext elements (e.g. frames in a keyframe)
    virtual void updateSubContext();

  public:

    //ds singleton configuration
    static constexpr Count minimum_landmark_age = 3;
    static constexpr Count minimum_image_age    = 1;

  protected:

    const Identifier _index;
    Status _status;
    Frame* _previous = 0;
    Frame* _next     = 0;

    //ds frame point generation
    FramePointPtrVector _points;

    //ds spatials
    TransformMatrix3D _frame_to_local_map      = TransformMatrix3D::Identity();
    TransformMatrix3D _local_map_to_frame      = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world          = TransformMatrix3D::Identity();
    TransformMatrix3D _world_to_robot          = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world_odometry = TransformMatrix3D::Identity();

    //ds TODO refactor to support arbitrary camera constellations
    const Camera* _camera_left   = 0;
    const Camera* _camera_right  = 0;

    //ds TODO refactor to support arbitrary number of rgb/depth image combinations
    IntensityImage _intensity_image_left;
    IntensityImage _intensity_image_right;
    const real _maximum_depth_close = 0;

    const LocalMap* _local_map = 0;
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
}
