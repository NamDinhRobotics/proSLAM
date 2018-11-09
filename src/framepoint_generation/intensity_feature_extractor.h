#pragma once
#include "types/definitions.h"



namespace proslam {

class IntensityFeatureExtractor {
public:


protected:

  //! @brief grid of detectors (equally distributed over the image with size=number_of_detectors_per_dimension*number_of_detectors_per_dimension)
  cv::Ptr<cv::FastFeatureDetector>** _detectors = nullptr;
  real** _detector_thresholds                   = nullptr;

  //! @brief number of detectors
  //! @brief the same for all image streams
  uint32_t _number_of_detectors;

  //! @brief image region for each detector
  //! @brief the same for all image streams
  cv::Rect** _detector_regions = nullptr;
};
} //namespace proslam
