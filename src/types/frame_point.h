#pragma once
#include "definitions.h"

namespace proslam {
  
  class Landmark;
  class Frame;
  class FramePoint {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds possible framepoint status
    enum Status {Created, Confirmed, Persistent};

    //ds prohibit default construction
    FramePoint() = delete;

  //ds object handling: instantiation controlled by frame class (factory)
  protected:

    //ds construct a new framepoint in the provided frame
    FramePoint(const cv::KeyPoint& keypoint_left_,
               const cv::Mat& descriptor_left_,
               const cv::KeyPoint& keypoint_right_,
               const cv::Mat& descriptor_right_,
               Frame* frame_);

  public:

    inline const Identifier index() const {return _index;}

    inline Frame* frame() {return _frame;}
    inline const Frame* frame() const {return _frame;}
    void setFrame(Frame* frame_) {_frame = frame_;}

    //inline FramePoint* parent()  {return _parent;}
    inline const FramePoint* previous() const {return _previous;}
    void setPrevious(FramePoint* previous_);

    inline FramePoint* root()  {return _root;}
    inline const FramePoint* root() const {return _root;}
    void setRoot(FramePoint* root_) {_root = root_;}

    inline Landmark* landmark() {return _landmark;}
    inline const Landmark* landmark() const {return _landmark;}
    void setLandmark(Landmark* landmark_) {_landmark = landmark_;}

    inline const real depth() const {return _depth;}
    void setDepth(const real& depth_meters_) {_depth = depth_meters_;}

    inline void setIsClose(const bool& is_close_) {_is_close = is_close_;}
    inline const bool isClose() const {return _is_close;}

    inline const Count age() const {return _age;}
    void setAge(const Count& age_) {_age = age_;}

    inline const PointCoordinates reprojectionCoordinatesLeft() const {return _reprojection_coordinates_left;}
    inline const PointCoordinates reprojectionCoordinatesRight() const {return _reprojection_coordinates_right;}
    void setReprojectionCoordinatesLeft(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_left = reprojection_coordinates_; }
    void setReprojectionCoordinatesRight(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_right = reprojection_coordinates_;}

    inline const Status status() const {return _status;}
    void setStatus(Status s) {_status=s;}

    inline const PointCoordinates& imageCoordinatesLeft() const {return _image_coordinates_left;}
    inline const PointCoordinates& imageCoordinatesRight() const {return _image_coordinates_right;}

    inline const PointCoordinates cameraCoordinatesLeft() const {return _camera_coordinates_left;}
    void setCameraCoordinatesLeft(const PointCoordinates& coordinates_) {_camera_coordinates_left = coordinates_;}

    inline const PointCoordinates robotCoordinates() const {return _robot_coordinates;}
    void setRobotCoordinates(const PointCoordinates& robot_coordinates_) {_robot_coordinates = robot_coordinates_;}

    inline const cv::KeyPoint& keypointLeft() const {return _keypoint_left;}
    inline const cv::KeyPoint& keypointRight() const {return _keypoint_right;}
    inline const cv::Mat& descriptorLeft() const {return _descriptor_left;}
    inline const cv::Mat& descriptorRight() const {return _descriptor_right;}
    inline const real disparity() const {return _disparity;}

  protected:

    //ds fixed properties set at construction
    const Identifier _index;
    FramePoint* _previous = 0; //< point in the previous image
    FramePoint* _root     = 0; //< point in the image where it was first detected
    Frame* _frame         = 0; //< frame to which the point belongs

    //ds feature based
    const cv::KeyPoint _keypoint_left; //ds keypoint obtained upon feature extraction
    const cv::KeyPoint _keypoint_right;
    const cv::Mat _descriptor_left;    //ds descriptor of feature extraction
    const cv::Mat _descriptor_right;
    const real _disparity = -1;

    //ds spatial properties
    PointCoordinates _image_coordinates_left   = PointCoordinates::Zero(); //ds detection image coordinates x/y/1
    PointCoordinates _image_coordinates_right  = PointCoordinates::Zero(); //ds detection image coordinates for additional camera (e.g. stereo)
    PointCoordinates _camera_coordinates_left  = PointCoordinates::Zero();
    PointCoordinates _robot_coordinates        = PointCoordinates::Zero(); //ds 3D point in robot pose frame
    PointCoordinates _reprojection_coordinates_left  = PointCoordinates::Zero(); //ds if a point is a landmark, these are its reprojected coordinates x/y/1
    PointCoordinates _reprojection_coordinates_right = PointCoordinates::Zero(); //ds reprojected coordinates for additional camera (e.g. stereo)
    real _depth = -1; //ds depth value in meters in camera coordinate frame

    //ds control
    Landmark* _landmark = 0; //ds landmark, if any
    bool _is_close      = false;
    Count _age          = 0; //ds number of frames where the point was continuously detected
    Status _status      = FramePoint::Created;

  //ds grant access to factory
  friend Frame;

  private:
    static Identifier _instances;
  };

  typedef std::vector<FramePoint, Eigen::aligned_allocator<FramePoint> > FramePointVector;
  typedef std::vector<FramePoint*> FramePointPtrVector;

}
