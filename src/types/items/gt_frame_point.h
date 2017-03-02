#pragma once
#include "types/gt_defs.h"

namespace gslam {
  
  class Landmark;
  class Frame;
  class FramePoint {
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    enum Status {Created, Confirmed, Persistent};

    //ds prohibit default construction
    FramePoint() = delete;

  //ds object handling: instantiation controlled by frame class (factory)
  protected:

    //ds RGBD: construction based on keypoint and descriptor (new detection)
    FramePoint(const cv::KeyPoint& keypoint_, const cv::Mat& descriptor_, Frame* frame_);

    //ds RGBD: construction based on existing, previous point with descriptor update
    FramePoint(const cv::KeyPoint& keypoint_, const cv::Mat& descriptor_, Frame* frame_, FramePoint* previous_point_);

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

    void setDepth(const gt_real& depth_meters_);
    inline bool hasDepth() const {return _has_depth;}

    inline const bool hasDepthByVision() const {return _has_depth_by_vision;}
    void setDepthByVision(const gt_real& depth_meters_);

    inline const Count age() const {return _age;}
    inline void setAge(const Count& age_) {_age = age_;}

    inline const PointCoordinates reprojectionCoordinates() const {return _reprojection_coordinates;}
    inline const PointCoordinates reprojectionCoordinatesExtra() const {return _reprojection_coordinates_extra;}
    inline void setReprojectionCoordinates(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates = reprojection_coordinates_; }
    inline void setReprojectionCoordinatesExtra(const PointCoordinates& reprojection_coordinates_extra_) {_reprojection_coordinates_extra = reprojection_coordinates_extra_;}

    inline const Status status() const {return _status;}
    inline void setStatus(Status s) {_status=s;}

    inline const bool isStereo() const {return _is_stereo;}

    inline const PointCoordinates& imageCoordinates() const { return _image_coordinates; }
    inline const PointCoordinates& imageCoordinatesExtra() const { return _image_coordinates_extra; }

    inline const PointCoordinates robotCoordinates() const {return _robot_coordinates;}
    inline void setRobotCoordinates(const PointCoordinates& robot_coordinates_) {_robot_coordinates = robot_coordinates_;}

    inline const cv::KeyPoint& keypoint() const {return _keypoint;}
    inline const cv::KeyPoint& keypointExtra() const {return _keypoint_extra;}
    inline const cv::Mat& descriptor() const {return _descriptor;}
    inline const cv::Mat& descriptorExtra() const {return _descriptor_extra;}
    inline const gt_real disparity() const {return _disparity;}
    inline const gt_real depth() const {return _depth;}

  protected:

    //ds fixed properties set at construction
    const Identifier _index;
    FramePoint* _previous = 0; //< point in the previous image
    FramePoint* _root     = 0; //< point in the image where it was first detected
    Frame* _frame         = 0; //< frame to which the point belongs

    //ds feature based
    const cv::KeyPoint _keypoint; //ds keypoint obtained upon feature extraction
    const cv::KeyPoint _keypoint_extra;
    const cv::Mat _descriptor;    //ds descriptor of feature extraction
    const cv::Mat _descriptor_extra;
    const gt_real _disparity = -1;

    //ds spatial properties
    PointCoordinates _image_coordinates        = PointCoordinates::Zero(); //ds detection image coordinates x/y/1
    PointCoordinates _image_coordinates_extra  = PointCoordinates::Zero(); //ds detection image coordinates for additional camera (e.g. stereo)
    PointCoordinates _robot_coordinates        = PointCoordinates::Zero(); //ds 3D point in robot pose frame
    PointCoordinates _reprojection_coordinates = PointCoordinates::Zero(); //ds if a point is a landmark, these are its reprojected coordinates x/y/1
    PointCoordinates _reprojection_coordinates_extra = PointCoordinates::Zero(); //ds reprojected coordinates for additional camera (e.g. stereo)
    gt_real _depth = -1; //ds depth value in meters in camera coordinate frame

    //ds control
    Landmark* _landmark  = 0; //< landmark, if any
    bool _has_depth      = false; //< true if the point has a valid depth
    bool _has_depth_by_vision = false; //< true if the point has depth, derived by some other context
    bool _is_stereo      = false; //< true if the point was generated by a stereo tracker
    Count _age           = 0; // number of frames where the point was continuously detected
    Status _status       = FramePoint::Created;

  //ds grant access to factory
  friend Frame;

  private:
    static Identifier _instances;
  };

  typedef std::vector<FramePoint, Eigen::aligned_allocator<FramePoint> > FramePointVector;
  typedef std::vector<FramePoint*> FramePointPtrVector;

} //namespace gtracker
