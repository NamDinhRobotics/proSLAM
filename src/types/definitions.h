#pragma once
#include <iostream>
#include <stdint.h>
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
  #include <opencv2/xfeatures2d.hpp>
#else
  #error OpenCV version not supported
#endif

#include "srrg_system_utils/system_utils.h"
#include "srrg_hbst_types/binary_tree.hpp"
#include "srrg_types/types.hpp"

namespace proslam {

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  //ds descriptor bit width
#ifndef SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS
  #define SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS 256
#endif
  #define DESCRIPTOR_SIZE_BYTES SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS/8
#ifndef SRRG_PROSLAM_DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE
  #define SRRG_PROSLAM_DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE 25
#endif
  #define DESCRIPTOR_NORM cv::NORM_HAMMING

  //ds adjust floating point precision
  typedef double real;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURATION// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  //ds existential types
  typedef Eigen::Matrix<real, 3, 1> PointCoordinates;
  typedef Eigen::Matrix<real, 3, 1> ImageCoordinates;
  typedef Eigen::Matrix<real, 3, 1> PointColorRGB;
  typedef Eigen::Matrix<real, 3, 3> CameraMatrix;
  typedef Eigen::Matrix<real, 3, 4> ProjectionMatrix;
  typedef Eigen::Transform<real, 3, Eigen::Isometry> TransformMatrix3D;
  typedef Eigen::Matrix<real, 6, 1> TransformVector3D;
  typedef Eigen::Quaternion<real> Quaternion;
  typedef uint64_t Identifier;
  typedef uint64_t Index;
  typedef uint64_t Count;
  typedef cv::Mat IntensityImage;
  typedef cv::Mat DepthImage;
  typedef std::vector<IntensityImage> IntensityImageVector;
  typedef std::vector<DepthImage> DepthImageVector;
  typedef std::pair<PointCoordinates, PointColorRGB> PointDrawable;

  //ds generic types
  typedef Eigen::Matrix<real, 2, 1> Vector2;  
  typedef Eigen::Matrix<real, 2, 3> Matrix2;
  typedef Eigen::Matrix<real, 3, 1> Vector3;  
  typedef Eigen::Matrix<real, 4, 1> Vector4;
  typedef Eigen::Matrix<real, 3, 3> Matrix3;
  typedef Eigen::Matrix<real, 4, 4> Matrix4;
  typedef Eigen::Matrix<real, 6, 6> Matrix6;
  typedef Eigen::Matrix<real, 5, 1> Vector5;
  typedef Eigen::Matrix<real, 6, 1> Vector6;  
  typedef Eigen::Matrix<real, 1, 6> Matrix1_6;  
  typedef Eigen::Matrix<real, 3, 6> Matrix3_6;
  typedef Eigen::Matrix<real, 4, 6> Matrix4_6;
  typedef Eigen::Matrix<real, 2, 6> Matrix2_6;
  typedef Eigen::Matrix<real, 2, 3> Matrix2_3;
  typedef Eigen::Matrix<real, 6, 3> Matrix6_3;
  typedef Eigen::Matrix<real, 6, 4> Matrix6_4;
  typedef Eigen::Matrix<real, 3, 4> Matrix3_4;
  typedef Eigen::Matrix<real, 4, 3> Matrix4_3;

  //ds HBST: readability
  typedef srrg_hbst::BinaryMatchable<SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS> HBSTMatchable;
  typedef srrg_hbst::BinaryNode<HBSTMatchable, 50, real> HBSTNode;
  typedef srrg_hbst::BinaryTree<HBSTNode, SRRG_PROSLAM_DESCRIPTOR_MAXIMUM_HAMMING_DISTANCE> HBSTTree;

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

  //ds extended boolean
  enum ThreeValued{False, True, Unknown};

  //ds timing
  #define CREATE_CHRONOMETER(NAME) \
  protected: double _time_consumption_seconds_##NAME = 0; \
  public: const double getTimeConsumptionSeconds_##NAME() const {return _time_consumption_seconds_##NAME;}
  #define CHRONOMETER_START(NAME) const double time_start_seconds_##NAME = srrg_core::getTime();
  #define CHRONOMETER_STOP(NAME) _time_consumption_seconds_##NAME += srrg_core::getTime()-time_start_seconds_##NAME;

  //ds "release assertions"
  #define ASSERT(CONDITION) \
    if (!CONDITION) { \
      throw std::runtime_error("flying polar buffalo error"); \
    }

  //ds generic logging macro - called by each implemented logging function
  #define LOG_GENERIC(LOG_LEVEL, EXPRESSION) \
      std::cerr << srrg_core::getTimestamp() << "|" << LOG_LEVEL << "|"; \
      EXPRESSION;

//ds conditional log levels, 1: INFO
#if SRRG_PROSLAM_LOG_LEVEL == 1

  //ds display all available logging except debug
  #define LOG_DEBUG(EXPRESSION)
  #define LOG_INFO(EXPRESSION) \
    LOG_GENERIC("INFO   ", EXPRESSION)
  #define LOG_WARNING(EXPRESSION) \
    LOG_GENERIC("WARNING", EXPRESSION)
  #define LOG_ERROR(EXPRESSION) \
    LOG_GENERIC("ERROR  ", EXPRESSION)

//ds 2: WARNING
#elif SRRG_PROSLAM_LOG_LEVEL == 2

  //ds display warnings and errors but no info and no debug
  #define LOG_DEBUG(EXPRESSION)
  #define LOG_INFO(EXPRESSION)
  #define LOG_WARNING(EXPRESSION) \
    LOG_GENERIC("WARNING", EXPRESSION)
  #define LOG_ERROR(EXPRESSION) \
    LOG_GENERIC("ERROR  ", EXPRESSION)

//ds 3: ERROR
#elif SRRG_PROSLAM_LOG_LEVEL == 3

  //ds display only errors
  #define LOG_DEBUG(EXPRESSION)
  #define LOG_INFO(EXPRESSION)
  #define LOG_WARNING(EXPRESSION)
  #define LOG_ERROR(EXPRESSION) \
    LOG_GENERIC("ERROR  ", EXPRESSION)

//ds default or 0: DEBUG
#else

  //ds default = 0, all logging active
  #define LOG_DEBUG(EXPRESSION) \
    LOG_GENERIC("DEBUG  ", EXPRESSION)
  #define LOG_INFO(EXPRESSION) \
    LOG_GENERIC("INFO   ", EXPRESSION)
  #define LOG_WARNING(EXPRESSION) \
    LOG_GENERIC("WARNING", EXPRESSION)
  #define LOG_ERROR(EXPRESSION) \
    LOG_GENERIC("ERROR  ", EXPRESSION)

#endif
};
