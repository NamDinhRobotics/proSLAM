#include "depth_framepoint_generator.h"

namespace proslam {

DepthFramePointGenerator::DepthFramePointGenerator(DepthFramePointGeneratorParameters* parameters_): BaseFramePointGenerator(parameters_),
                                                                                                     _parameters(parameters_),
                                                                                                     _camera_right(0) {
  LOG_INFO(std::cerr << "DepthFramePointGenerator::DepthFramePointGenerator|constructed" << std::endl)
}

//ds the stereo camera setup must be provided
void DepthFramePointGenerator::configure(){
  LOG_INFO(std::cerr << "DepthFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_right);

  //ds update base
  BaseFramePointGenerator::configure();

  //ds info
  LOG_INFO(std::cerr << "DepthFramePointGenerator::configure|configured" << std::endl)
}

//ds cleanup of dynamic structures
DepthFramePointGenerator::~DepthFramePointGenerator() {
  LOG_INFO(std::cerr << "DepthFramePointGenerator::~DepthFramePointGenerator|destroyed" << std::endl)
}

void DepthFramePointGenerator::_computeDepthMap(const cv::Mat& right_depth_image) {
  if (right_depth_image.type()!=CV_16UC1){
    throw std::runtime_error("depth tracker requires a 16bit mono image to encode depth");
  }

  //ds allocate new space map and initialize fields
  _space_map_left_meters.create(_number_of_rows_image,_number_of_cols_image, CV_32FC3);
  for (int32_t row = 0; row < _number_of_rows_image; ++row) {
    for (int32_t col = 0; col < _number_of_cols_image; ++col) {
      _space_map_left_meters.at<cv::Vec3f>(row, col) = cv::Vec3f(0,0,_maximum_reliable_depth_far_meters);
    }
  }

  _row_map.create(_number_of_rows_image,_number_of_cols_image, CV_16SC1);
  _row_map=-1;

  _col_map.create(_number_of_rows_image,_number_of_cols_image, CV_16SC1);
  _col_map=-1;

  const Matrix3 inverse_camera_matrix_right=_camera_right->cameraMatrix().inverse();
  const float _depth_pixel_to_meters=1e-3;

  TransformMatrix3D right_to_left_transform=_camera_left->robotToCamera()*_camera_right->cameraToRobot();
  for (int32_t r=0; r<_number_of_rows_image; ++r){
    const unsigned short* raw_depth=right_depth_image.ptr<const unsigned short>(r);
    for (int32_t c=0; c<_number_of_cols_image; ++c, raw_depth++){
      if (!*raw_depth)
        continue;
      // retrieve depth
      const float depth_right_meters=(*raw_depth)*_depth_pixel_to_meters;
      // retrieve point in right camers, meters
      Vector3 point_in_right_camera_meters=inverse_camera_matrix_right*Vector3(c*depth_right_meters, r*depth_right_meters,depth_right_meters);
      // map the point to the left camera
      Vector3 point_in_left_camera_meters=right_to_left_transform*point_in_right_camera_meters;
      // if beyond camera, discard
      const real depth_left_meters=point_in_left_camera_meters.z();
      if (depth_left_meters<=0)
        continue;
      // project to image coordinates
      Vector3 point_in_left_camera_pixels=_camera_left->cameraMatrix()*point_in_left_camera_meters;
      point_in_left_camera_pixels *= 1./point_in_left_camera_pixels.z();

      // round to int
      const int32_t dest_r=round(point_in_left_camera_pixels.y());
      const int32_t dest_c=round(point_in_left_camera_pixels.x());

      // if outside skip
      if (dest_r<0 ||
          dest_r>=_number_of_rows_image ||
          dest_c<0 ||
          dest_c>=_number_of_cols_image)
        continue;

      // do z buffering and update indices
      cv::Vec3f& dest_space=_space_map_left_meters.at<cv::Vec3f>(dest_r, dest_c);

      if (dest_space[2] > depth_left_meters){
        dest_space=cv::Vec3f(point_in_left_camera_meters.x(),
                             point_in_left_camera_meters.y(),
                             point_in_left_camera_meters.z());
        _row_map.at<unsigned short>(dest_r, dest_c)=r;
        _col_map.at<unsigned short>(dest_r, dest_c)=c;
      }
    }
  }
}

void DepthFramePointGenerator::initialize(Frame* frame_, const bool& extract_features_) {
  //ds TODO
}

//ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
void DepthFramePointGenerator::compute(Frame* frame_) {
  assert(frame_->intensityImageRight().type() == CV_16UC1);

  //ds detect new features
  detectKeypoints(frame_->intensityImageLeft(), frame_->keypointsLeft());

  CHRONOMETER_START(depth_map_generation)
  _computeDepthMap(frame_->intensityImageRight());
  CHRONOMETER_STOP(depth_map_generation)

  //ds extract descriptors for detected features
  computeDescriptors(frame_->intensityImageLeft(), frame_->keypointsLeft(), frame_->descriptorsLeft());

  //ds prepare and execute stereo keypoint search
  CHRONOMETER_START(depth_assignment)
  computeCoordinatesFromDepth(frame_, frame_->keypointsLeft(), frame_->descriptorsLeft());
  CHRONOMETER_STOP(depth_assignment)
}

//ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
void DepthFramePointGenerator::computeCoordinatesFromDepth(Frame* frame_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
  _number_of_available_points = 0;
  frame_->points().resize(keypoints_.size());

  for (size_t i=0; i<keypoints_.size(); i++) {
    const cv::KeyPoint & keypoint_left=keypoints_[i];
    const cv::Mat& descriptor_left=descriptors_.row(i);
    const Index r_left=keypoint_left.pt.y;
    const Index c_left=keypoint_left.pt.x;
    const cv::Vec3f& p=_space_map_left_meters.at<const cv::Vec3f>(r_left, c_left);
    if (p[2]>=_maximum_reliable_depth_far_meters)
      continue;


    cv::KeyPoint keypoint_right=keypoint_left;
    keypoint_right.pt.x=_col_map.at<short>(r_left,c_left);
    keypoint_right.pt.y=_row_map.at<short>(r_left,c_left);

    const PointCoordinates camera_coordinates=PointCoordinates(p[0], p[1], p[2]); // compute

    //ds add to framepoint map
    frame_->points()[_number_of_available_points] = frame_->createFramepoint(keypoint_left,
                                                                             descriptor_left,
                                                                             keypoint_right,
                                                                             descriptor_left,
                                                                             camera_coordinates);
    ++_number_of_available_points;
  }
  frame_->points().resize(_number_of_available_points);
}
}
