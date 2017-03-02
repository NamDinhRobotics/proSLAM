#pragma once
#include "../contexts/gt_frame.h"

namespace gslam {

  class LandmarkItem;
  class Appearance {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Appearance(const LandmarkItem* item_,
               const cv::Mat& descriptor_cv_): item(item_),
                                               descriptor_cv(descriptor_cv_),
                                               descriptor(_getDescriptor(descriptor_cv_))/*,
                                               descriptor_bow(descriptor.to_string())*/{}

  public:

    const LandmarkItem* item;
    const cv::Mat descriptor_cv;
    const HBSTMatchable::BinaryDescriptor descriptor;
    //const DBoW2::FBrief::TDescriptor descriptor_bow;

  //ds converters: should be removed!
  private:

    inline const HBSTMatchable::BinaryDescriptor _getDescriptor(const cv::Mat& descriptor_cv_) {
      HBSTMatchable::BinaryDescriptor binary_descriptor(DESCRIPTOR_SIZE_BITS);
      for (uint32_t byte_index = 0 ; byte_index < DESCRIPTOR_SIZE_BYTES; ++byte_index) {

        //ds get minimal datafrom cv::mat
        const uchar value = descriptor_cv_.at<uchar>(byte_index);

        //ds get bitstring
        for (uint8_t v = 0; v < 8; ++v) {
          binary_descriptor[byte_index*8+v] = (value >> v) & 1;
        }
      }
      return binary_descriptor;
    }

  }; //class Appearance

  typedef std::vector<const Appearance*> AppearancePtrVector;

} //namespace gtracker
