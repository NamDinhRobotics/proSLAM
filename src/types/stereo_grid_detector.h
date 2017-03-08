#include "contexts/frame.h"
#include "types/camera.h"

namespace proslam {

  class ExceptionTriangulation: public std::exception {
  public:

    ExceptionTriangulation(const std::string& what_): _what(what_){}
    ~ExceptionTriangulation(){}

  public:

    virtual const char* what() const throw() {return _what.c_str();}

  private:

    const std::string _what;
  };

  class StereoGridDetector {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds exported datatypes
    public:

      struct KeypointWithDescriptor {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        cv::KeyPoint keypoint;
        cv::Mat descriptor;
        int32_t r;
        int32_t c;
      };

      struct TriangulatedPoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool is_available = false;
        cv::KeyPoint keypoint_left;
        cv::KeyPoint keypoint_right;
        cv::Mat descriptor_left;
        cv::Mat descriptor_right;
        PointCoordinates camera_coordinates_left;
      };

    //ds object handling
    public:

      StereoGridDetector(const Camera* camera_left_,
                         const Camera* camera_right_);
      ~StereoGridDetector();
      StereoGridDetector() = delete;

    //ds access
    public:

      const Count triangulate(const Frame* frame_);

      const PointCoordinates getCoordinatesInCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_);

//      int32_t** keypointIndexMapLeft() {return _keypoint_index_map_left;}
//      int32_t** keypointIndexMapRight() {return _keypoint_index_map_right;}
      TriangulatedPoint** framepointMap() {return _triangulation_map;}
      const Count numberOfRowsImage() const {return _number_of_rows_image;}
      const Count numberOfColsImage() const {return _number_of_cols_image;}
      const int32_t maximumTrackingMatchingDistance() const {return _maximum_tracking_matching_distance;}
      void setTargetNumberOfPoints(const Count& target_number_of_points_) {_target_number_of_points = target_number_of_points_;}
      void setDetectorThreshold(const int32_t& detector_threshold_) {_detector_threshold = detector_threshold_;}
      void setDetectorThresholdMaximum(const int32_t& detector_threshold_maximum_) {_detector_threshold_maximum = detector_threshold_maximum_;}
      void setDetectorThresholdMinimum(const int32_t& detector_threshold_minimum_) {_detector_threshold_minimum = detector_threshold_minimum_;}
      void setMaximumMatchingDistanceTriangulation(const real& maximum_matching_distance_triangulation_) {_maximum_matching_distance_triangulation = maximum_matching_distance_triangulation_;}

    //ds public settings UGLY UGLY
    public:

      static real maximum_depth_close;
      static real maximum_depth_far;

    //ds settings
    protected:

      //ds detection/generic
      const Count _number_of_rows_image = 0;
      const Count _number_of_cols_image = 0;
      Count _target_number_of_points    = 750;
      int32_t _detector_threshold = 10;
      int32_t _detector_threshold_minimum = 10;
      int32_t _detector_threshold_maximum = 50;
      int32_t _maximum_tracking_matching_distance = 50;
      const int32_t _tracking_matching_distance_threshold_maximum = 50;
      const int32_t _tracking_matching_distance_threshold_minimum = 25;

      //ds feature density normalization
      const Count _bin_size         = 4;
      const Count _number_of_bins_u = 0;
      const Count _number_of_bins_v = 0;
      cv::KeyPoint** _bin_map_left;
//      cv::KeyPoint** _bin_map_right;

      //ds triangulation
      real _maximum_matching_distance_triangulation = 50;
      const real _triangulation_F           = 0;
      const real _triangulation_Finverse    = 0;
      const real _triangulation_Pu          = 0;
      const real _triangulation_Pv          = 0;
      const real _triangulation_DuR         = 0;
      const real _triangulation_DuR_flipped = 0;
      const real _baseline_meters           = 0;
      const real _baseline_factor           = 50;
      const real _minimum_disparity         = 1;
      int32_t** _keypoint_index_map_left;
      int32_t** _keypoint_index_map_right;
      TriangulatedPoint** _triangulation_map;

      //ds feature detection
#if CV_MAJOR_VERSION == 2
      std::shared_ptr<cv::FeatureDetector> _feature_detector = 0;
#elif CV_MAJOR_VERSION == 3
      cv::Ptr<cv::Feature2D> _feature_detector;
#else
      #error OpenCV version not supported
#endif

      //ds descriptor extraction
#if CV_MAJOR_VERSION == 2
      std::shared_ptr<cv::DescriptorExtractor> _descriptor_extractor = 0;
#elif CV_MAJOR_VERSION == 3
      const cv::Ptr<cv::Feature2D> _descriptor_extractor;
#else
      #error OpenCV version not supported
#endif

      //ds buffers
      std::vector<cv::KeyPoint> _keypoints_left;
      std::vector<cv::KeyPoint> _keypoints_right;
      cv::Mat _descriptors_left;
      cv::Mat _descriptors_right;
      std::vector<KeypointWithDescriptor> _keypoints_with_descriptor_left;
      std::vector<KeypointWithDescriptor> _keypoints_with_descriptor_right;
      std::vector<std::pair<KeypointWithDescriptor, KeypointWithDescriptor>> _stereo_keypoints;

    //ds helpers
    protected:

      void _binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_);

      CREATE_CHRONOMETER(feature_detection)
      CREATE_CHRONOMETER(descriptor_extraction)
      CREATE_CHRONOMETER(keypoint_pruning)
      CREATE_CHRONOMETER(point_triangulation)
  };
}
