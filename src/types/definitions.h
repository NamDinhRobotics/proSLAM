#pragma once
#include <iostream>
#include <stdint.h>
#include <set>
#include <vector>
#include <map>
#include <Eigen/Geometry>
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
#include "srrg_types/types.hpp"

namespace proslam {

  //ds pattern helper macros
  #define PROSLAM_CONCATENATE(A, B) A ## B

  //ds pattern macros
  #define PROSLAM_MAKE_PROCESSING_CLASS(CLASS_NAME) \
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
      CLASS_NAME(PROSLAM_CONCATENATE(CLASS_NAME, Parameters)* parameters_); \
      virtual void configure(); \
      virtual ~CLASS_NAME(); \
    private: PROSLAM_CONCATENATE(CLASS_NAME, Parameters)* _parameters; \
    public: PROSLAM_CONCATENATE(CLASS_NAME, Parameters)* parameters() {return _parameters;}

  #define PROSLAM_MAKE_PROCESSING_SUBCLASS(SUBCLASS_NAME, PARAMETERS_TYPE) \
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
      SUBCLASS_NAME(PARAMETERS_TYPE* parameters_); \
      virtual ~SUBCLASS_NAME();

  //ds descriptor bit width
#ifndef SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS
  #define SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS 256
#endif
  #define DESCRIPTOR_SIZE_BYTES SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS/8
  #define SRRG_PROSLAM_DESCRIPTOR_NORM cv::NORM_HAMMING

  //ds adjust floating point precision
  typedef double real;

  //ds existential types
  typedef Eigen::Matrix<real, 3, 1> PointCoordinates;
  typedef std::vector<PointCoordinates, Eigen::aligned_allocator<PointCoordinates>> PointCoordinatesVector;
  typedef Eigen::Matrix<real, 3, 1> ImageCoordinates;
  typedef std::vector<ImageCoordinates, Eigen::aligned_allocator<ImageCoordinates>> ImageCoordinatesVector;
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
  typedef std::vector<IntensityImage> IntensityImageVector;
  typedef cv::Mat DepthImage;
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
  typedef Eigen::Matrix<real, 1, 3> Matrix1_3;
  typedef Eigen::Matrix<real, 2, 3> Matrix2_3;
  typedef Eigen::Matrix<real, 6, 3> Matrix6_3;
  typedef Eigen::Matrix<real, 6, 4> Matrix6_4;
  typedef Eigen::Matrix<real, 3, 4> Matrix3_4;
  typedef Eigen::Matrix<real, 4, 3> Matrix4_3;

  //ds cv colors
  #define CV_COLOR_CODE_RANDOM cv::Scalar(rand()%255, rand()%255, rand()%255)
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

  //ds log levels
  enum LoggingLevel {Debug   = 0,
                     Info    = 1,
                     Warning = 2,
                     Error   = 3};

  //ds timing
  #define CREATE_CHRONOMETER(NAME) \
  protected: double _time_consumption_seconds_##NAME = 0; \
  public: const double getTimeConsumptionSeconds_##NAME() const {return _time_consumption_seconds_##NAME;}
  #define CHRONOMETER_START(NAME) const double time_start_seconds_##NAME = srrg_core::getTime();
  #define CHRONOMETER_STOP(NAME) _time_consumption_seconds_##NAME += srrg_core::getTime()-time_start_seconds_##NAME;

  //ds print functions
  #define        BAR "---------------------------------------------------------------------------------------------------------------------------------"
  #define DOUBLE_BAR "================================================================================================================================="

  //ds assertion handling
  #define ASSERT(ASSERTION, INFO) \
    assert(ASSERTION && INFO);

  //ds log level
#ifndef SRRG_PROSLAM_LOG_LEVEL
  #define SRRG_PROSLAM_LOG_LEVEL 2
#endif

  //ds generic logging macro - called by each implemented logging function
  #define LOG_GENERIC(LOGGING_LEVEL, LOGGING_LEVEL_DESCRIPTION, EXPRESSION) \
    std::cerr << srrg_core::getTimestamp() << "|" << LOGGING_LEVEL_DESCRIPTION << "|"; \
    EXPRESSION;

  //ds log levels (defined at compile time)
#if SRRG_PROSLAM_LOG_LEVEL > 2
  #define LOG_DEBUG(EXPRESSION) \
    LOG_GENERIC(LoggingLevel::Debug, "DEBUG  ", EXPRESSION)
#else
  #define LOG_DEBUG(EXPRESSION)
#endif

#if SRRG_PROSLAM_LOG_LEVEL > 1
  #define LOG_INFO(EXPRESSION) \
    LOG_GENERIC(LoggingLevel::Info, "INFO   ", EXPRESSION)
#else
  #define LOG_INFO(EXPRESSION)
#endif

#if SRRG_PROSLAM_LOG_LEVEL > 0
  #define LOG_WARNING(EXPRESSION) \
    LOG_GENERIC(LoggingLevel::Warning, "WARNING", EXPRESSION)
#else
  #define LOG_WARNING(EXPRESSION)
#endif

  #define LOG_ERROR(EXPRESSION) \
    LOG_GENERIC(LoggingLevel::Error, "ERROR  ", EXPRESSION)

}
