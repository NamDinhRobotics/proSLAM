#include "depth_framepoint_generator.h"

namespace proslam {

  DepthFramePointGenerator::DepthFramePointGenerator() {
    _camera_right=0;
  }
  
  //ds the stereo camera setup must be provided
  void DepthFramePointGenerator::setup(){
    assert(_camera_right);

    _target_number_of_detected_keypoints=700;
   BaseFramePointGenerator::setup();
   _maximum_depth_near_meters = 6;
   _maximum_depth_far_meters = 30;
   
    //ds info
    std::cerr << "DepthFramePointGenerator::DepthFramePointGenerator|maximum depth tracking close (m): " << _maximum_depth_near_meters << std::endl;
    std::cerr << "DepthFramePointGenerator::DepthFramePointGenerator|maximum depth tracking far (m): " << _maximum_depth_far_meters << std::endl;
    std::cerr << "DepthFramePointGenerator::DepthFramePointGenerator|constructed" << std::endl;
  }

  //ds cleanup of dynamic structures
  DepthFramePointGenerator::~DepthFramePointGenerator() {
  }

  void DepthFramePointGenerator::_computeDepthMap(const cv::Mat& right_depth_image) {
    if (right_depth_image.type()!=CV_16UC1){
      throw std::runtime_error("depth tracker requires a 16bit mono image to encode depth");
    }

    //ds allocate new space map and initialize fields
    _space_map_left_meters.create(_number_of_rows_image,_number_of_cols_image, CV_32FC3);
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      for (Index col = 0; col < _number_of_cols_image; ++col) {
        _space_map_left_meters.at<cv::Vec3f>(row, col) = cv::Vec3f(0,0,_maximum_depth_far_meters);
      }
    }

    _row_map.create(_number_of_rows_image,_number_of_cols_image, CV_16SC1);
    _row_map=-1;
    
    _col_map.create(_number_of_rows_image,_number_of_cols_image, CV_16SC1);
    _col_map=-1;

    const Matrix3 inverse_camera_matrix_right=_camera_right->cameraMatrix().inverse();
    const float _depth_pixel_to_meters=1e-3;
    
    TransformMatrix3D right_to_left_transform=_camera_left->robotToCamera()*_camera_right->cameraToRobot();
    for (Count r=0; r<_number_of_rows_image; ++r){
      const unsigned short* raw_depth=right_depth_image.ptr<const unsigned short>(r);
      for (Count c=0; c<_number_of_cols_image; ++c, raw_depth++){
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
	const Count dest_r=round(point_in_left_camera_pixels.y());
	const Count dest_c=round(point_in_left_camera_pixels.x());
	
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

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  void DepthFramePointGenerator::compute(Frame* frame_) {
    assert(frame_->intensityImageRight().type() == CV_16UC1);

    //ds detect new features
    detectKeypoints(frame_->intensityImageLeft(), _keypoints_left);

    CHRONOMETER_START(depth_map_generation)
    _computeDepthMap(frame_->intensityImageRight());
    CHRONOMETER_STOP(depth_map_generation)

    //ds extract descriptors for detected features
    extractDescriptors(frame_->intensityImageLeft(), _keypoints_left, _descriptors_left);

    //ds prepare and execute stereo keypoint search
    CHRONOMETER_START(depth_assignment)
    computeCoordinatesFromDepth(frame_);
    CHRONOMETER_STOP(depth_assignment)
  }

  //ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
  void DepthFramePointGenerator::computeCoordinatesFromDepth(Frame* frame_) {
    _number_of_available_points = 0;
    

    for (size_t i=0; i<_keypoints_left.size(); i++) {
      const cv::KeyPoint & keypoint_left=_keypoints_left[i];
      const cv::Mat& descriptor_left=_descriptors_left.row(i);
      const Index r_left=keypoint_left.pt.y;
      const Index c_left=keypoint_left.pt.x;
      const cv::Vec3f& p=_space_map_left_meters.at<const cv::Vec3f>(r_left, c_left);
      if (p[2]>=_maximum_depth_far_meters)
	continue;

      assert(_framepoints_in_image[r_left][c_left] == 0);

      cv::KeyPoint keypoint_right=keypoint_left;
      keypoint_right.pt.x=_col_map.at<short>(r_left,c_left);
      keypoint_right.pt.y=_row_map.at<short>(r_left,c_left);

      const PointCoordinates camera_coordinates=PointCoordinates(p[0], p[1], p[2]); // compute 
      //ds add to framepoint map
      _framepoints_in_image[r_left][c_left] = frame_->create(keypoint_left,
							     descriptor_left,
							     keypoint_right,
							     descriptor_left,
							     camera_coordinates);
      ++_number_of_available_points;
    }
  }
}
