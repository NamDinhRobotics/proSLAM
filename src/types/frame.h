#pragma once
#include "base_context.h"
#include "camera.h"
#include "frame_point.h"

namespace proslam {
  
  //ds forward declarations
  class LocalMap;
  class WorldMap;

  //ds this class encapsulates all data gained from the processing of a stereo image pair
  class Frame: public BaseContext {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds defines one of the two tracker states
    enum Status {Localizing, Tracking};

  //ds object handling
  protected:

    //ds frame construction in the WorldMap context
    Frame(const WorldMap* context_,
          Frame* previous_,
          Frame* next_,
          const TransformMatrix3D& robot_to_world_,
          const real& maximum_depth_close_);

    //ds deep copy constructor
    Frame(Frame* frame_);

    //ds FramePoints cleanup
    virtual ~Frame();

    //ds prohibt default construction
    Frame() = delete;

  //ds getters/setters
  public:

    const Identifier& identifier() const {return _identifier;}

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
    virtual void setRobotToWorld(const TransformMatrix3D& robot_to_world_);
    inline const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}
    
    inline const TransformMatrix3D& frameToLocalMap() const {return _frame_to_local_map;}
    inline const TransformMatrix3D& localMapToFrame() const {return _local_map_to_frame;}

    //ds visualization only
    void setRobotToWorldGroundTruth(const TransformMatrix3D& robot_to_world_ground_truth_) {_robot_to_world_ground_truth = robot_to_world_ground_truth_;}
    const TransformMatrix3D& robotToWorldGroundTruth() const {return _robot_to_world_ground_truth;}

    inline const FramePointPointerVector& points() const {return _points;}
    inline FramePointPointerVector& points() {return _points;}

    //ds request new framepoint with optional link to a previous point (track)
    FramePoint* create(const cv::KeyPoint& keypoint_left_,
                       const cv::Mat& descriptor_left_,
                       const cv::KeyPoint& keypoint_right_,
                       const cv::Mat& descriptor_right_,
                       const PointCoordinates& camera_coordinates_left_,
                       FramePoint* previous_point_ = 0);

    inline const IntensityImage& intensityImageLeft() const {return _intensity_image_left;}
    inline void setIntensityImageLeft(const IntensityImage& intensity_image_)  {_intensity_image_left = intensity_image_.clone();}

    inline const IntensityImage& intensityImageRight() const {return _intensity_image_right;}
    inline void setIntensityImageRight(const IntensityImage& intensity_image_)  {_intensity_image_right = intensity_image_.clone();}

    inline Status status() const {return _status;}
    inline void setStatus(const Status& status_) {_status = status_;}

    inline const real maximumDepthClose() const {return _maximum_depth_close;}

    void setLocalMap(const LocalMap* local_map_);
    const LocalMap* localMap() const {return _local_map;}
    void setIsLocalMapAnchor(const bool& is_local_map_anchor_) {_is_local_map_anchor = is_local_map_anchor_;}
    const bool isLocalMapAnchor() const {return _is_local_map_anchor;}

    const Count countPoints(const Count min_age_,
		                        const ThreeValued has_landmark_ = Unknown) const;

    void releaseImages();
    void releasePoints();

  //ds configuration attributes
  public:

    //ds singleton configuration
    static constexpr Count minimum_landmark_age = 3;
    static constexpr Count minimum_image_age    = 1;

  //ds attributes
  protected:

    const Identifier _identifier;
    Status _status   = Localizing;
    Frame* _previous = 0;
    Frame* _next     = 0;

    //ds frame point generation
    FramePointPointerVector _points;

    //ds spatials
    TransformMatrix3D _frame_to_local_map = TransformMatrix3D::Identity();
    TransformMatrix3D _local_map_to_frame = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world     = TransformMatrix3D::Identity();
    TransformMatrix3D _world_to_robot     = TransformMatrix3D::Identity();

    //ds stereo camera configuration affiliated with this frame
    const Camera* _camera_left   = 0;
    const Camera* _camera_right  = 0;

    //ds to support arbitrary number of rgb/depth image combinations
    IntensityImage _intensity_image_left;
    IntensityImage _intensity_image_right;
    const real _maximum_depth_close = 0;

    //ds link to a local map if the frame is part of one
    const LocalMap* _local_map = 0;
    bool _is_local_map_anchor  = false;

    //ds visualization only
    TransformMatrix3D _robot_to_world_ground_truth = TransformMatrix3D::Identity();

    //ds access
    friend class WorldMap;
    friend class FramePtrMap;

    //ds class specific
    private:

      //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
      static Count _instances;
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
