#pragma once
#include "relocalization/local_map_correspondence.h"
#include "base_aligner.h"

namespace proslam {

  //ds class that implements an aligner interface between two local maps
  class BaseLocalMapAligner: public BaseAligner {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseLocalMapAligner() {}

    BaseLocalMapAligner(const real& error_delta_for_convergence_): BaseAligner(error_delta_for_convergence_) {}

    BaseLocalMapAligner(const real& error_delta_for_convergence_,
                        const real& maximum_error_kernel_): BaseAligner(error_delta_for_convergence_,
                                                                        maximum_error_kernel_) {}

    BaseLocalMapAligner(const real& error_delta_for_convergence_,
                        const real& maximum_error_kernel_,
                        const real& damping_): BaseAligner(error_delta_for_convergence_,
                                                           maximum_error_kernel_,
                                                           damping_) {}

    BaseLocalMapAligner(const real& error_delta_for_convergence_,
                        const real& maximum_error_kernel_,
                        const real& damping_,
                        const uint64_t& maximum_number_of_iterations_): BaseAligner(error_delta_for_convergence_,
                                                                                    maximum_error_kernel_,
                                                                                    damping_,
                                                                                    maximum_number_of_iterations_) {}

    virtual ~BaseLocalMapAligner() {};

  //ds functionality
  public:

    virtual void initialize(LocalMapCorrespondence* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity()) = 0;

  //ds setters/getters
  public:

    inline const TransformMatrix3D& currentToReference() const {return _current_to_reference;}
    inline void setMinimumNumberOfInliers(const Count& minimum_number_of_inliers_) {_minimum_number_of_inliers = minimum_number_of_inliers_;}
    inline void setMinimumInlierRatio(const real& minimum_inlier_ratio_) {_minimum_inlier_ratio = minimum_inlier_ratio_;}
    
  protected:

    //ds context
    LocalMapCorrespondence* _context = 0;

    //ds objective
    TransformMatrix3D _current_to_reference = TransformMatrix3D::Identity();

    Count _minimum_number_of_inliers = 100;
    real _minimum_inlier_ratio       = 0.75;
  };
}
