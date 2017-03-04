#pragma once
#include "types/definitions.h"

namespace gslam {

  class AlignerFactory;
  template<uint64_t states, uint64_t dimension>
  class BaseAligner {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //ds readability
    public:

      typedef Eigen::Matrix<gt_real, states, states> AlignmentMatrix;
      typedef Eigen::Matrix<gt_real, states, 1> AlignmentVector;
      typedef Eigen::Matrix<gt_real, dimension, dimension> InformationMatrix;
      typedef Eigen::Matrix<gt_real, dimension, states> JacobianMatrix;

    //ds object handling
    protected:

      //ds direct instantiation prohibited
      BaseAligner() {};
      BaseAligner(const gt_real& error_delta_for_convergence_): _error_delta_for_convergence(error_delta_for_convergence_) {};
      BaseAligner(const gt_real& error_delta_for_convergence_,
                  const gt_real& maximum_error_kernel_): _error_delta_for_convergence(error_delta_for_convergence_),
                                                         _maximum_error_kernel(maximum_error_kernel_) {};
      BaseAligner(const gt_real& error_delta_for_convergence_,
                  const gt_real& maximum_error_kernel_,
                  const gt_real& damping_): _error_delta_for_convergence(error_delta_for_convergence_),
                                            _maximum_error_kernel(maximum_error_kernel_),
                                            _damping(damping_) {};
      BaseAligner(const gt_real& error_delta_for_convergence_,
                  const gt_real& maximum_error_kernel_,
                  const gt_real& damping_,
                  const uint64_t& maximum_number_of_iterations_): _error_delta_for_convergence(error_delta_for_convergence_),
                                                                  _maximum_error_kernel(maximum_error_kernel_),
                                                                  _damping(damping_),
                                                                  _maximum_number_of_iterations(maximum_number_of_iterations_) {};
      virtual ~BaseAligner() {};

    //ds access
    public:

      //ds initialize aligner with minimal entity TODO enforce this for all sub classes once purified
      virtual void init(BaseContext* context_, const TransformMatrix3D& initial_guess_) = 0;

      //ds linearize the system: to be called inside oneRound
      virtual void linearize(const bool& ignore_outliers_) = 0;

      //ds solve alignment problem for one round: calling linearize
      virtual void oneRound(const bool& ignore_outliers_) = 0;

      //ds solve alignment problem until convergence is reached
      virtual void converge() = 0;

      //ds getters/setters
      const std::vector<gt_real>& errors() const {return _errors;}
      const std::vector<bool>& inliers() const {return _inliers;}
      const uint64_t numberOfInliers() const {return _number_of_inliers;}
      const uint64_t numberOfOutliers() const {return _number_of_outliers;}
      const uint64_t numberOfCorrespondences() const {return _number_of_inliers+_number_of_outliers;}
      const gt_real errorDeltaForConvergence() const {return _error_delta_for_convergence;}
      const Count maximumNumberOfIterations() const {return _maximum_number_of_iterations;}
      void setDamping(const gt_real& damping_) {_damping = damping_;}
      const gt_real damping() const {return _damping;}
      const gt_real totalError() const {return _total_error;}
      const bool hasSystemConverged() const {return _has_system_converged;}
      void setMaximumErrorKernel(const gt_real& maximum_error_kernel_) {_maximum_error_kernel = maximum_error_kernel_;}
      const gt_real maximumErrorKernel() const {return _maximum_error_kernel;}
      const Matrix6& informationMatrix() const {return _information_matrix;}

    //ds aligner specific
    protected:

      //ds control
      std::vector<gt_real> _errors;
      std::vector<bool> _inliers;
      uint64_t _number_of_inliers  = 0;
      uint64_t _number_of_outliers = 0;

      //ds convergence criteria
      gt_real _error_delta_for_convergence = 1e-5;
      gt_real _maximum_error_kernel        = 1.0;
      gt_real _damping                     = 1.0;
      Count _maximum_number_of_iterations  = 100;
      bool _has_system_converged           = false;

      //ds general LS parameters
      gt_real _total_error     = 0.0;
      AlignmentMatrix _H       = AlignmentMatrix::Zero();
      AlignmentVector _b       = AlignmentVector::Zero();
      InformationMatrix _omega = InformationMatrix::Identity();
      JacobianMatrix _jacobian = JacobianMatrix::Zero();
      AlignmentMatrix _information_matrix = AlignmentMatrix::Identity();

    //ds grant access to factory: ctor/dtor
    friend AlignerFactory;

  }; //class BaseAligner

  //ds aligner types
  enum AlignerType6_3 {
    uvd,
    xuvd,
    xyz,
    pxyz
  };
  enum AlignerType6_4 {
    stereouv
  };

  //ds readability
  typedef BaseAligner<6, 3> BaseAligner6_3;
  typedef BaseAligner<6, 4> BaseAligner6_4;



} //namespace gslam
