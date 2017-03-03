#pragma once
#include "types/contexts/gt_world_context.h"
#include "types/stereo_grid_detector.h"

namespace gslam {

  class ExceptionTracking: public std::exception {
  public:

    ExceptionTracking(const std::string& what_): _what(what_){}
    ~ExceptionTracking(){}

  public:

    virtual const char* what() const throw() {return _what.c_str();}

  private:

    const std::string _what;
  };

  class ExceptionTriangulation: public ExceptionTracking {
  public:

    ExceptionTriangulation(const std::string& what_): ExceptionTracking(what_){}
    ~ExceptionTriangulation(){}
  };

  class ExceptionMatching: public ExceptionTracking {
  public:

    ExceptionMatching(const std::string& what_): ExceptionTracking(what_){}
    ~ExceptionMatching(){}
  };

  struct MatchTriangulation {
      MatchTriangulation(const cv::KeyPoint& keypoint_,
                         const cv::Mat& descriptor_): keypoint(keypoint_),
                                                      descriptor(descriptor_) {}
      MatchTriangulation(const MatchTriangulation& match_): keypoint(match_.keypoint),
                                                            descriptor(match_.descriptor) {}

      const cv::KeyPoint keypoint;
      const cv::Mat descriptor;
  };

  class TrackerSVI {

  //ds object handling
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TrackerSVI(WorldContext* context_,
               const Camera* camera_left_,
               const Camera* camera_right_);
    ~TrackerSVI();

  //ds access
  public:

    const TransformMatrix3D addImage(const Camera* camera_left_,
                                     const cv::Mat& intensity_image_left_,
                                     const Camera* camera_right_,
                                     const cv::Mat& intensity_image_right_,
                                     const TransformMatrix3D& initial_guess_transform_previous_to_current_ = TransformMatrix3D::Identity(),
                                     const Identifier& sequence_number_raw_ = 0);
    
    const TrackingContext* trackingContext() const {return _context->currentTrackingContext();}
    const TransformMatrix3D robotToWorld() const {return _context->currentTrackingContext()->robotToWorldPrevious();}
    BaseAligner6_4* aligner() {return _aligner;}
    std::shared_ptr<StereoGridDetector> gridSensor() {return _grid_sensor;}
    void setOdometryRobotToWorld(const TransformMatrix3D& robot_to_world_) {if (_context->currentTrackingContext()->currentFrame()) _context->currentTrackingContext()->currentFrame()->setRobotToWorldOdometry(robot_to_world_);}
    const Count totalNumberOfTrackedPoints() const {return _total_number_of_tracked_points;}
    const Count totalNumberOfLandmarksClose() const {return _total_number_of_landmarks_close;}
    const Count totalNumberOfLandmarksFar() const {return _total_number_of_landmarks_far;}
    void setMaximumPixelDistanceTrackingMinimum(const int32_t& maximum_pixel_distance_tracking_minimum_) {_pixel_distance_tracking_minimum = maximum_pixel_distance_tracking_minimum_;}

  //ds helpers
  protected:

    void trackFeatures(Frame* previous_frame_);
    void extractFeatures(Frame* frame_);
    void predictCvPoints(std::vector<cv::Point2f>& predictions_left,
                         std::vector<cv::Point2f>& predictions_right,
                         Frame* previous_frame_,
                         const Frame* current_frame_) const;
    void triangulate(const Frame* frame_);
    void updateLandmarks(Frame* frame_);
    void recoverPoints(Frame* frame_);

  protected:

    //ds control
    WorldContext* _context = 0;
    Frame::Status _status          = Frame::Localizing;
    Frame::Status _status_previous = Frame::Localizing;

    //ds configuration
    Count _minimum_number_of_landmarks_to_track = 5;
    Count _number_of_tracked_landmarks_far      = 0;
    Count _number_of_tracked_landmarks_close    = 0;
    Count _number_of_potential_points  = 0;
    Count _number_of_tracked_points    = 0;

    //ds 3d point retrieval
    std::shared_ptr<StereoGridDetector> _grid_sensor;

    //ds track recovery
    Count _number_of_lost_points           = 0;
    Count _number_of_lost_points_recovered = 0;
    std::vector<FramePoint*> _lost_points;

    //ds feature tracking criteria
    int32_t _pixel_distance_tracking         = 0;
    int32_t _pixel_distance_tracking_maximum = 50;
    int32_t _pixel_distance_tracking_minimum = 25;
    const int32_t _range_point_tracking      = 2;

    //ds pose solving
    StereoUVAligner* _aligner = 0;

    //ds wrapped access TODO macromize
    public: const double getTimeConsumptionSeconds_feature_detection() const {return _grid_sensor->getTimeConsumptionSeconds_feature_detection();}
    public: const double getTimeConsumptionSeconds_keypoint_pruning() const {return _grid_sensor->getTimeConsumptionSeconds_keypoint_pruning();}
    public: const double getTimeConsumptionSeconds_descriptor_extraction() const {return _grid_sensor->getTimeConsumptionSeconds_descriptor_extraction();}
    public: const double getTimeConsumptionSeconds_point_triangulation() const {return _grid_sensor->getTimeConsumptionSeconds_point_triangulation();}

    //ds stats
    CREATE_CHRONOMETER(tracking)
    CREATE_CHRONOMETER(track_creation)
    CREATE_CHRONOMETER(pose_optimization)
    CREATE_CHRONOMETER(landmark_optimization)
    CREATE_CHRONOMETER(point_recovery)
    Count _total_number_of_tracked_points  = 0;
    Count _total_number_of_landmarks_close = 0;
    Count _total_number_of_landmarks_far   = 0;
  };
}
