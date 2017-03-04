#pragma once

#include <queue>
#include <set>
#include <vector>
#include <map>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION == 2
  //ds no specifics
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgcodecs.hpp>
  #include <opencv2/highgui.hpp>
  #include <opencv2/imgproc.hpp>
  #include <opencv2/features2d.hpp>
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/calib3d.hpp>
  #include <opencv/cv.h>
#else
  #error OpenCV version not supported
#endif
#include <iostream>
#include <stdint.h>
#include "srrg_system_utils/system_utils.h"
#include "srrg_hbst_types_core/BinaryTree.hpp"
#include "contexts/base_context.h"



namespace gslam{

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  //ds descriptor bit width
  #define DESCRIPTOR_SIZE_BITS 256
  #define DESCRIPTOR_SIZE_BYTES DESCRIPTOR_SIZE_BITS/8
  #define DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE 25
  #define DESCRIPTOR_NORM cv::NORM_HAMMING

  //ds adjust for floating point precision
  typedef double gt_real;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURATION// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  //ds existential types
  typedef Eigen::Matrix<gt_real, 3, 1> PointCoordinates;
  typedef Eigen::Matrix<gt_real, 3, 1> PointColorRGB;
  typedef Eigen::Matrix<gt_real, 3, 3> CameraMatrix;
  typedef Eigen::Matrix<gt_real, 3, 4> ProjectionMatrix;
  typedef Eigen::Transform<gt_real, 3, Eigen::Isometry> TransformMatrix3D;
  typedef Eigen::Matrix<gt_real, 6, 1> TransformVector3D;
  typedef uint64_t Identifier;
  typedef uint64_t Index;
  typedef uint64_t Count;
  typedef cv::Mat IntensityImage;
  typedef cv::Mat DepthImage;
  typedef std::vector<IntensityImage> IntensityImageVector;
  typedef std::vector<DepthImage> DepthImageVector;
  typedef std::pair<PointCoordinates, PointColorRGB> PointDrawable;

  //ds generic types
  typedef Eigen::Matrix<gt_real, 2, 1> Vector2;  
  typedef Eigen::Matrix<gt_real, 2, 3> Matrix2;
  typedef Eigen::Matrix<gt_real, 3, 1> Vector3;  
  typedef Eigen::Matrix<gt_real, 4, 1> Vector4;
  typedef Eigen::Matrix<gt_real, 3, 3> Matrix3;
  typedef Eigen::Matrix<gt_real, 4, 4> Matrix4;
  typedef Eigen::Matrix<gt_real, 6, 6> Matrix6;
  typedef Eigen::Matrix<gt_real, 6, 1> Vector6;  
  typedef Eigen::Matrix<gt_real, 1, 6> Matrix1_6;  
  typedef Eigen::Matrix<gt_real, 3, 6> Matrix3_6;
  typedef Eigen::Matrix<gt_real, 4, 6> Matrix4_6;
  typedef Eigen::Matrix<gt_real, 2, 6> Matrix2_6;
  typedef Eigen::Matrix<gt_real, 2, 3> Matrix2_3;
  typedef Eigen::Matrix<gt_real, 6, 3> Matrix6_3;
  typedef Eigen::Matrix<gt_real, 6, 4> Matrix6_4;

  //ds HBST
  typedef srrg_hbst::BinaryMatchable<DESCRIPTOR_SIZE_BITS> HBSTMatchable;
  typedef srrg_hbst::BinaryNode<HBSTMatchable, 50, gt_real> HBSTNode;
  typedef srrg_hbst::BinaryTree<HBSTNode, DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE> HBSTTree;

  //ds cv colors
  #define CV_COLOR_CODE_GREEN cv::Scalar(0, 200, 0)
  #define CV_COLOR_CODE_BLUE cv::Scalar(255, 0, 0)
  #define CV_COLOR_CODE_DARKBLUE cv::Scalar(150, 0, 0)
  #define CV_COLOR_CODE_RED cv::Scalar(0, 0, 255)
  #define CV_COLOR_CODE_YELLOW cv::Scalar(0, 255, 255)
  #define CV_COLOR_CODE_WHITE cv::Scalar(255, 255, 255)
  #define CV_COLOR_CODE_BLACK cv::Scalar(0, 0, 0)
  #define CV_COLOR_CODE_ORANGE cv::Scalar(0, 150, 255)
  #define CV_COLOR_CODE_DARKGREEN cv::Scalar(0, 150, 0)
  #define CV_COLOR_CODE_BROWN cv::Scalar(0, 100, 175)
  #define CV_COLOR_CODE_VIOLETT cv::Scalar(255, 0, 255)
  #define CV_COLOR_CODE_DARKVIOLETT cv::Scalar(150, 0, 150)

  //ds good point counting - TODO rethink
  enum ThreeValued{
      False,
      True,
      Unknown
  };

  //ds timing
  #define CREATE_CHRONOMETER(NAME) \
  protected: double _time_consumption_seconds_##NAME = 0; \
  public: const double getTimeConsumptionSeconds_##NAME() const {return _time_consumption_seconds_##NAME;}
  #define CHRONOMETER_START(NAME) const double time_start_seconds_##NAME = srrg_core::getTime();
  #define CHRONOMETER_STOP(NAME) _time_consumption_seconds_##NAME += srrg_core::getTime()-time_start_seconds_##NAME;

  #define ASSERT(CONDITION) \
    if (!CONDITION) { \
      throw std::runtime_error("flying polar buffalo error"); \
    }
};
