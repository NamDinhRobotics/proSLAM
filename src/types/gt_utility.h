#pragma once
#include "types/gt_defs.h"

namespace gslam {

  //ds TODO this class MUST be purged
  class Utility {

  public:

    static const Eigen::Matrix< gt_real, 3, 1 > toOrientationRodrigues( const Eigen::Matrix< gt_real, 3, 3 >& p_matRotation )
    {
        //ds orientation
        cv::Mat vecOrientation;

        //ds fill the matrix
        cv::Rodrigues( toCVMatrix( p_matRotation ), vecOrientation );

        //ds return in Eigen format
        return fromCVVector< gt_real, 3 >( vecOrientation );
    }

    static const cv::Mat_< gt_real > toCVMatrix( const Eigen::Matrix< gt_real, 3, 3 >& p_matEigen )
    {
        //ds allocate cv vector
        cv::Mat_< gt_real > matCV( 3, 3 );

        //ds fill the vector (column major)
        for( uint32_t u = 0; u < 3; ++u )
        {
            for( uint32_t v = 0; v < 3; ++v )
            {
                matCV.at< gt_real >( u, v ) = p_matEigen( u, v );
            }
        }

        return matCV;
    }

    template< typename tType, uint32_t uRows > static const Eigen::Matrix< tType, uRows, 1 > fromCVVector( const cv::Vec< tType, uRows >& p_vecCV )
    {
        //ds allocate eigen matrix
        Eigen::Matrix< tType, uRows, 1 > vecEigen;

        //ds fill the vector (column major)
        for( uint32_t u = 0; u < uRows; ++u )
        {
            vecEigen( u ) = p_vecCV( u );
        }

        return vecEigen;
    }

    //ds TODO remove
    static const cv::Mat getCvMat(const AppearancePtrVector& descriptors) {
      assert(0 < descriptors.size());
      cv::Mat descriptors_cv;
      for(const Appearance* descriptor_query: descriptors) {
        descriptors_cv.push_back(descriptor_query->descriptor);
      }
      return descriptors_cv;
    }

    //ds TODO remove
    static const HBSTNode::BinaryMatchableVector getMatchables(const AppearancePtrVector& appearances_) {
      assert(appearances_.size() > 0);
      HBSTNode::BinaryMatchableVector matchables(appearances_.size());

      //ds copy raw data
      for (Index index_appearance = 0; index_appearance < appearances_.size(); ++index_appearance) {
        matchables[index_appearance] = new HBSTMatchable(index_appearance, appearances_[index_appearance]->descriptor);
      }
      return matchables;
    }

    //ds TODO remove
    static const AppearancePtrVector getAppearances(const FramePointPtrVector& framepoints_) {
      assert(framepoints_.size() > 0);
      AppearancePtrVector appearances(framepoints_.size());

      //ds copy raw data
      Count number_of_added_appearances = 0;
      for (Index index_appearance = 0; index_appearance < appearances.size(); ++index_appearance) {
        FramePoint* point = framepoints_[index_appearance];
        if (point->landmark() != 0) {
          appearances[number_of_added_appearances] = new Appearance(new LandmarkItem(point->landmark(), point->robotCoordinates()), point->descriptor());
          ++number_of_added_appearances;
        }
      }

      appearances.resize(number_of_added_appearances);
      return appearances;
    }

//    //ds TODO remove
//    static const std::vector<DBoW2::FBrief::TDescriptor> getDescriptorsBoW(const AppearancePtrVector& appearances_) {
//      assert(appearances_.size() > 0);
//      std::vector<DBoW2::FBrief::TDescriptor> descriptors_BoW(appearances_.size());
//
//      //ds copy raw data
//      for (Index index_appearance = 0; index_appearance < appearances_.size(); ++index_appearance) {
//        descriptors_BoW[index_appearance] = appearances_[index_appearance]->descriptor_bow;
//      }
//      return descriptors_BoW;
//    }

  }; //class Utility
} //namespace gtracker
