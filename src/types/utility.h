#pragma once
#include "types/definitions.h"

namespace proslam {

  //ds TODO this class MUST be purged
  class Utility {

  public:

    static const Eigen::Matrix< real, 3, 1 > toOrientationRodrigues( const Eigen::Matrix< real, 3, 3 >& p_matRotation )
    {
        //ds orientation
        cv::Mat vecOrientation;

        //ds fill the matrix
        cv::Rodrigues( toCVMatrix( p_matRotation ), vecOrientation );

        //ds return in Eigen format
        return fromCVVector< real, 3 >( vecOrientation );
    }

    static const cv::Mat_< real > toCVMatrix( const Eigen::Matrix< real, 3, 3 >& p_matEigen )
    {
        //ds allocate cv vector
        cv::Mat_< real > matCV( 3, 3 );

        //ds fill the vector (column major)
        for( uint32_t u = 0; u < 3; ++u )
        {
            for( uint32_t v = 0; v < 3; ++v )
            {
                matCV.at< real >( u, v ) = p_matEigen( u, v );
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

  }; //class Utility
} //namespace gtracker
