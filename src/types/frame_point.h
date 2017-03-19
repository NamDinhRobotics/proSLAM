#pragma once
#include "definitions.h"

namespace proslam {
  
  class Landmark;
  class Frame;
  class FramePoint {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds public object handling
  public:

    enum Status {Created, Confirmed, Persistent};

    //ds prohibit default construction
    FramePoint() = delete;

  //ds object handling: instantiation controlled by frame class (factory)
  protected:

    //ds STEREO: construction based on keypoint and descriptor (new detection)
    FramePoint(const cv::KeyPoint& keypoint_left_,
               const cv::Mat& descriptor_left_,
               const cv::KeyPoint& keypoint_right_,
               const cv::Mat& descriptor_right_,
               Frame* frame_);

    //ds STEREO: construction based on existing, previous point with descriptor update
    FramePoint(const cv::KeyPoint& keypoint_left_,
               const cv::Mat& descriptor_left_,
               const cv::KeyPoint& keypoint_right_,
               const cv::Mat& descriptor_right_,
               Frame* frame_,
               FramePoint* previous_point_);

  public:

    inline const Identifier index() const {return _index;}

    inline Frame* frame() {return _frame;}
    inline const Frame* frame() const {return _frame;}
    inline void setFrame(Frame* frame_) {_frame = frame_;}

    //inline FramePoint* parent()  {return _parent;}
    inline const FramePoint* previous() const {return _previous;}
    inline void setPrevious(FramePoint* parent_) {_previous = parent_;}

    inline FramePoint* root()  {return _root;}
    inline const FramePoint* root() const {return _root;}
    inline void setRoot(FramePoint* root_) {_root = root_;}

    inline Landmark* landmark() {return _landmark;}
    inline const Landmark* landmark() const {return _landmark;}
    inline void setLandmark(Landmark* landmark_) {_landmark = landmark_;}

    inline void setDepth(const real& depth_meters_) {assert(depth_meters_ > 0.0); _depth = depth_meters_;}
    inline const real depth() const {return _depth;}

    inline void setIsClose(const bool& is_close_) {_is_close = is_close_;}
    inline const bool isClose() const {return _is_close;}

    inline const Count age() const {return _age;}
    inline void setAge(const Count& age_) {_age = age_;}

    inline const PointCoordinates reprojectionCoordinatesLeft() const {return _reprojection_coordinates_left;}
    inline const PointCoordinates reprojectionCoordinatesRight() const {return _reprojection_coordinates_right;}
    inline void setReprojectionCoordinatesLeft(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_left = reprojection_coordinates_; }
    inline void setReprojectionCoordinatesRight(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_right = reprojection_coordinates_;}

    inline const Status status() const {return _status;}
    inline void setStatus(Status s) {_status=s;}

    inline const PointCoordinates& imageCoordinatesLeft() const { return _image_coordinates_left; }
    inline const PointCoordinates& imageCoordinatesRight() const { return _image_coordinates_right; }

    inline const PointCoordinates robotCoordinates() const {return _robot_coordinates;}
    inline void setRobotCoordinates(const PointCoordinates& robot_coordinates_) {_robot_coordinates = robot_coordinates_;}

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
    PointCoordinates _robot_coordinates        = PointCoordinates::Zero(); //ds 3D point in robot pose frame
    PointCoordinates _reprojection_coordinates_left  = PointCoordinates::Zero(); //ds if a point is a landmark, these are its reprojected coordinates x/y/1
    PointCoordinates _reprojection_coordinates_right = PointCoordinates::Zero(); //ds reprojected coordinates for additional camera (e.g. stereo)
    real _depth = -1; //ds depth value in meters in camera coordinate frame

    //ds control
    Landmark* _landmark  = 0; //ds landmark, if any
    bool _is_close       = false;
    Count _age           = 0; //ds number of frames where the point was continuously detected
    Status _status       = FramePoint::Created;

  //ds grant access to factory
  friend Frame;

  private:
    static Identifier _instances;
  };

  typedef std::vector<FramePoint, Eigen::aligned_allocator<FramePoint> > FramePointVector;
  typedef std::vector<FramePoint*> FramePointPtrVector;

}
