#pragma once
#include "camera.h"
#include "frame_point.h"

namespace proslam {
  
  //ds forward declarations
  class LocalMap;
  class WorldMap;

  //ds this class encapsulates all data gained from the processing of a stereo image pair
  class Frame {
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
          const real& maximum_depth_near_);

    //ds deep clone constructor - without incrementing the identifier!
    Frame(Frame* frame_);

    //ds FramePoints cleanup
    virtual ~Frame() {releasePoints();}

    //ds prohibit default construction
    Frame() = delete;

  //ds getters/setters
  public:

    //ds unique identifier for a frame instance (exists once in memory)
    const Identifier& identifier() const {return _identifier;}

    inline Frame* previous() {return _previous;}
    inline const Frame* previous() const {return _previous;}
    void setPrevious(Frame* previous_) {_previous = previous_;}

    inline Frame* next()  {return _next;}
    inline const Frame* next() const {return _next;}
    void setNext(Frame* next_) {_next = next_;}

    inline const Camera* cameraLeft() const {return _camera_left;}
    void setCameraLeft(const Camera* camera_) {_camera_left = camera_;}

    inline const Camera* cameraRight() const {return _camera_right;}
    void setCameraRight(const Camera* camera_) {_camera_right = camera_;}

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

    //ds this criteria is used for the decision of creating a landmark or not from a track of framepoints
    const Count& minimumTrackLengthForLandmarkCreation() const {return _minimum_track_length_for_landmark_creation;}

    //ds request a new framepoint instance with an optional link to a previous point (track)
    FramePoint* create(const cv::KeyPoint& keypoint_left_,
                       const cv::Mat& descriptor_left_,
                       const cv::KeyPoint& keypoint_right_,
                       const cv::Mat& descriptor_right_,
                       const PointCoordinates& camera_coordinates_left_,
                       FramePoint* previous_point_ = 0);

    inline const IntensityImage& intensityImageLeft() const {return _intensity_image_left;}
    void setIntensityImageLeft(const IntensityImage& intensity_image_)  {_intensity_image_left = intensity_image_.clone();}

    inline const IntensityImage& intensityImageRight() const {return _intensity_image_right;}
    void setIntensityImageRight(const IntensityImage& intensity_image_)  {_intensity_image_right = intensity_image_.clone();}

    inline const Status& status() const {return _status;}
    void setStatus(const Status& status_) {_status = status_;}

    //ds the maximum allowed depth for framepoints to become classified as near - everything above is far
    inline const real maximumDepthNear() const {return _maximum_depth_near;}

    void setLocalMap(const LocalMap* local_map_) {_local_map = local_map_;}
    inline const LocalMap* localMap() const {return _local_map;}
    void setFrameToLocalMap(const TransformMatrix3D& frame_to_local_map_) {_frame_to_local_map = frame_to_local_map_; _local_map_to_frame = _frame_to_local_map.inverse();}
    void setIsLocalMapAnchor(const bool& is_local_map_anchor_) {_is_local_map_anchor = is_local_map_anchor_;}
    inline const bool isLocalMapAnchor() const {return _is_local_map_anchor;}

    //ds get a quick overview of the overall point status in the frame
    const Count countPoints(const Count& min_track_length_,
		                        const ThreeValued& has_landmark_ = Unknown) const;

    //ds free open cv images
    void releaseImages();

    //ds free all point instances
    void releasePoints();

    //ds update framepoint world coordinates
    void updatePoints();

  //ds attributes
  protected:

    //ds unique identifier for a landmark (exists once in memory)
    const Identifier _identifier;

    //ds tracker status at the time of creation of this instance
    Status _status = Localizing;

    //ds links to preceding and subsequent instances
    Frame* _previous = 0;
    Frame* _next     = 0;

    //ds contained framepoints
    FramePointPointerVector _points;

    //ds this criteria is used for the decision of creating a landmark or not from a track of framepoints
    const Count _minimum_track_length_for_landmark_creation = 3;

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

    //ds the maximum allowed depth for framepoints to become classified as near - everything above is far
    const real _maximum_depth_near;

    //ds link to a local map if the frame is part of one
    const LocalMap* _local_map = 0;
    bool _is_local_map_anchor  = false;

    //ds access
    friend class WorldMap;
    friend class FramePointerMap;

    //ds visualization only
    TransformMatrix3D _robot_to_world_ground_truth = TransformMatrix3D::Identity();

    //ds class specific
    private:

      //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
      static Count _instances;
  };

  typedef std::vector<Frame*> FramePointerVector;
  typedef std::pair<Identifier, Frame*> FramePointerMapElement;

  class FramePointerMap: public std::map<Identifier, Frame*>{
  public:
    Frame* get(const Identifier& identifier_);
    void put(Frame* frame_);
    void replace(Frame* frame_);
  };
}
