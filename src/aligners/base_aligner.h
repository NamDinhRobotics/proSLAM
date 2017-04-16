#pragma once
#include "types/definitions.h"

namespace proslam {

  //ds base aligner class
  class BaseAligner {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    public:

      BaseAligner() {};
      BaseAligner(const real& error_delta_for_convergence_): _error_delta_for_convergence(error_delta_for_convergence_) {};
      BaseAligner(const real& error_delta_for_convergence_,
                  const real& maximum_error_kernel_): _error_delta_for_convergence(error_delta_for_convergence_),
                                                         _maximum_error_kernel(maximum_error_kernel_) {};
      BaseAligner(const real& error_delta_for_convergence_,
                  const real& maximum_error_kernel_,
                  const real& damping_): _error_delta_for_convergence(error_delta_for_convergence_),
                                            _maximum_error_kernel(maximum_error_kernel_),
                                            _damping(damping_) {};
      BaseAligner(const real& error_delta_for_convergence_,
                  const real& maximum_error_kernel_,
                  const real& damping_,
                  const uint64_t& maximum_number_of_iterations_): _error_delta_for_convergence(error_delta_for_convergence_),
                                                                  _maximum_error_kernel(maximum_error_kernel_),
                                                                  _damping(damping_),
                                                                  _maximum_number_of_iterations(maximum_number_of_iterations_) {};
      virtual ~BaseAligner() {};

    //ds access
    public:

      //ds linearize the system: to be called inside oneRound
      virtual void linearize(const bool& ignore_outliers_) = 0;

      //ds solve alignment problem for one round: calling linearize
      virtual void oneRound(const bool& ignore_outliers_) = 0;

      //ds solve alignment problem until convergence is reached
      virtual void converge() = 0;

      //ds getters/setters
      inline const std::vector<real>& errors() const {return _errors;}
      inline const std::vector<bool>& inliers() const {return _inliers;}
      inline const uint64_t numberOfInliers() const {return _number_of_inliers;}
      inline const uint64_t numberOfOutliers() const {return _number_of_outliers;}
      inline const uint64_t numberOfCorrespondences() const {return _number_of_inliers+_number_of_outliers;}
      inline const real errorDeltaForConvergence() const {return _error_delta_for_convergence;}
      inline const Count maximumNumberOfIterations() const {return _maximum_number_of_iterations;}
      inline void setDamping(const real& damping_) {_damping = damping_;}
      inline const real damping() const {return _damping;}
      inline const real totalError() const {return _total_error;}
      inline const bool hasSystemConverged() const {return _has_system_converged;}
      inline void setMaximumErrorKernel(const real& maximum_error_kernel_) {_maximum_error_kernel = maximum_error_kernel_;}
      inline const real maximumErrorKernel() const {return _maximum_error_kernel;}

    //ds aligner specific
    protected:

      //ds control
      std::vector<real> _errors;
      std::vector<bool> _inliers;
      uint64_t _number_of_inliers  = 0;
      uint64_t _number_of_outliers = 0;

      //ds convergence criteria
      real _error_delta_for_convergence   = 1e-5;
      real _maximum_error_kernel          = 1;
      real _damping                       = 1;
      Count _maximum_number_of_iterations = 100;
      bool _has_system_converged          = false;

      //ds general LS parameters
      real _total_error = 0;
  };

  //ds class that holds all generic (running) variables used in an aligner
  template<uint64_t states, uint64_t dimension>
  class AlignerWorkspace {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    public:

      //ds default workspace setup
      AlignerWorkspace() {}

      //ds default workspace tear-down
      ~AlignerWorkspace() {}

    //ds exported types
    public:

      typedef Eigen::Matrix<real, states, states> StateMatrix;
      typedef Eigen::Matrix<real, states, 1> StateVector;
      typedef Eigen::Matrix<real, dimension, dimension> DimensionMatrix;
      typedef Eigen::Matrix<real, dimension, states> JacobianMatrix;

    //ds workspace variables
    protected:
    
      StateMatrix _H                  = StateMatrix::Zero();
      StateVector _b                  = StateVector::Zero();
      DimensionMatrix _omega          = DimensionMatrix::Identity();
      JacobianMatrix _jacobian        = JacobianMatrix::Zero();
      StateMatrix _information_matrix = StateMatrix::Identity();
  };
}
