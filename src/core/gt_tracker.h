#pragma once
#include "types/contexts/gt_world_context.h"

namespace gslam {

  class Tracker {

  //ds object handling
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracker(WorldContext* context_);
    ~Tracker();

  //ds access
  public:

    const TransformMatrix3D addImage(Camera* camera_,
                                     const cv::Mat& intensity_image_,
                                     const cv::Mat& depth_image_,
                                     const Identifier& sequence_number_raw_,
                                     const TransformMatrix3D& estimate_world_previous_to_current_);
    
    const TrackingContext* trackingContext() const {return _context->currentTrackingContext();}

    //! minimum number of redetection of a feature before considering it stable
    inline int minImageAge() const {return Frame::minimum_image_age;}
    inline void setMinImageAge(const Count& ma) {std::cerr << "deprecated" << std::endl;}

    //! minimum number of redetection of a feature before considering it stable
    inline int minLandmarkAge() const {return Frame::minimum_landmark_age;}
    inline void setMinLandmarkAge(const Count& ma) {std::cerr << "deprecated" << std::endl;}

    //! minimum distance between the detected features
    inline void setMinFeatureDistance(const gt_real& d) {_min_feature_distance=d;}
    inline float minFeatureDistance() const {return _min_feature_distance;}

    //! maximum depth returned by the camera
    inline void setMaxDepth(const gt_real& max_depth) {_max_depth_meters=max_depth;}
    inline float maxDepth() const {return _max_depth_meters;}

    //! minimum number of reasonable inliers for the solver
    inline int minReprojectionInliers() const {return _min_reprojection_inliers;}
    inline void  setMinReprojectionInliers(const Count& min_reprojection_inliers) {_min_reprojection_inliers = min_reprojection_inliers;}

    const std::shared_ptr<std::list<TransformMatrix3D>> odometryMotions() const {return _odometry_motions;}

    const TransformMatrix3D getRobotToWorld() const {return _context->currentTrackingContext()->robotToWorldPrevious();}
    const BaseAligner6_3* aligner() const {return _aligner;}

  //ds helpers
  protected:

    void predictCvPoints(std::vector<cv::Point2f>& predictions_, const Frame* previous_frame_, const Frame* current_frame_) const;
    void trackFeatures(const Frame* previous_frame_) const;
    void extractFeatures();
    void updateDepth() const;
    void updateLandmarks() const;

  protected:

    //ds control
    WorldContext* _context;
    Frame::Status _status;

    //ds configuration
    gt_real _max_depth_meters;
    gt_real _min_feature_distance;
    Count _min_num_landmarks_to_track;
    Count _min_reprojection_inliers = 25;
    const gt_real _landmark_optimization_depth_weight = 100.0;
    const gt_real _infinity_depth_meters              = 100.0;

    //ds feature tracking
    cv::Size _optical_flow_win_size;
    cv::TermCriteria _optical_flow_termcrit;

#if CV_MAJOR_VERSION == 2
    cv::GFTTDetector _feature_detector;
#elif CV_MAJOR_VERSION == 3
    cv::Ptr<cv::Feature2D> _feature_detector;
#else
  #error OpenCV version not supported
#endif

    cv::Mat _feature_detection_mask;

    //ds pose solving
    UVDAligner* _aligner = 0;
    const Count _maximum_number_of_iterations_solver = 25;

    //ds external odometry
    std::shared_ptr<std::list<TransformMatrix3D>> _odometry_motions;

    CREATE_CHRONOMETER(overall)
  };
}
