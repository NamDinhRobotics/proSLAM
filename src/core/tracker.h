#pragma once
#include "types/stereo_triangulator.h"
#include "types/contexts/world_map.h"
#include "types/aligners/aligner_factory.h"

namespace proslam {

  class Tracker {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds object handling
  public:

    Tracker(const Camera* camera_left_, const Camera* camera_right_);
    ~Tracker();

  //ds access
  public:

    const TransformMatrix3D addImage(WorldMap* context_,
                                     const cv::Mat& intensity_image_left_,
                                     const cv::Mat& intensity_image_right_,
                                     const TransformMatrix3D& initial_guess_transform_previous_to_current_ = TransformMatrix3D::Identity());
    
    BaseAligner6_4* aligner() {return _aligner;}
    StereoTriangulator* preprocessor() {return _preprocessor;}
    const Count totalNumberOfTrackedPoints() const {return _total_number_of_tracked_points;}
    const Count totalNumberOfLandmarksClose() const {return _total_number_of_landmarks_close;}
    const Count totalNumberOfLandmarksFar() const {return _total_number_of_landmarks_far;}
    void setMaximumPixelDistanceTrackingMinimum(const int32_t& maximum_pixel_distance_tracking_minimum_) {_pixel_distance_tracking_minimum = maximum_pixel_distance_tracking_minimum_;}

  //ds helpers
  protected:

    void trackFeatures(Frame* previous_frame_, Frame* current_frame_);
    void extractFeatures(Frame* frame_);
    void getImageCoordinates(std::vector<ImageCoordinates>& predictions_left,
                             Frame* previous_frame_,
                             const Frame* current_frame_) const;
    void updateLandmarks(WorldMap* context_);
    void recoverPoints(WorldMap* context_);

  protected:

    //ds control
    Frame::Status _status          = Frame::Localizing;
    Frame::Status _status_previous = Frame::Localizing;

    //ds configuration
    Count _minimum_number_of_landmarks_to_track = 5;
    Count _number_of_tracked_landmarks_far      = 0;
    Count _number_of_tracked_landmarks_close    = 0;
    Count _number_of_potential_points  = 0;
    Count _number_of_tracked_points    = 0;
    const Camera* _camera_left  = 0;
    const Camera* _camera_right = 0;
    const int32_t _camera_rows  = 0;
    const int32_t _camera_cols  = 0;

    //ds 3d point retrieval
    StereoTriangulator* _preprocessor;

    //ds track recovery
    Count _number_of_lost_points           = 0;
    Count _number_of_lost_points_recovered = 0;
    std::vector<FramePoint*> _lost_points;

    //ds feature tracking criteria
    int32_t _pixel_distance_tracking           = 0;
    int32_t _pixel_distance_tracking_maximum   = 49; //7x7 pixels
    int32_t _pixel_distance_tracking_minimum   = 16; //4x4 pixels
    const int32_t _range_point_tracking        = 2;
    const int32_t _maximum_flow_pixels_squared = 150*150;

    //ds pose solving
    StereoUVAligner* _aligner = 0;

    //ds buffers
    std::vector<ImageCoordinates> _projected_image_coordinates_left;

    //ds wrapped access TODO macromize
    public: const double getTimeConsumptionSeconds_feature_detection() const {return _preprocessor->getTimeConsumptionSeconds_feature_detection();}
    public: const double getTimeConsumptionSeconds_keypoint_pruning() const {return _preprocessor->getTimeConsumptionSeconds_keypoint_pruning();}
    public: const double getTimeConsumptionSeconds_descriptor_extraction() const {return _preprocessor->getTimeConsumptionSeconds_descriptor_extraction();}
    public: const double getTimeConsumptionSeconds_point_triangulation() const {return _preprocessor->getTimeConsumptionSeconds_point_triangulation();}

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
