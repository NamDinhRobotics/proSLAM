#pragma once
#include "types/definitions.h"

namespace proslam {

//ds base aligner class
class BaseAligner {

//ds object handling
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! @brief default constructor
  //! @param[in] parameters_ target parameters handle
  BaseAligner(AlignerParameters* parameters_): _parameters(parameters_) {};

  //! @brief configuration method, called once all construction parameters are set
  virtual void configure() {}

  //! @brief default destructor
  virtual ~BaseAligner() {};

//ds functionality
public:

  //ds linearize the system: to be called inside oneRound
  virtual void linearize(const bool& ignore_outliers_) = 0;

  //ds solve alignment problem for one round: calling linearize
  virtual void oneRound(const bool& ignore_outliers_) = 0;

  //ds solve alignment problem until convergence is reached
  virtual void converge() = 0;

//ds getters/setters
public:

  inline const std::vector<real>& errors() const {return _errors;}
  inline const std::vector<bool>& inliers() const {return _inliers;}
  inline const Count numberOfInliers() const {return _number_of_inliers;}
  inline const Count numberOfOutliers() const {return _number_of_outliers;}
  inline const Count numberOfCorrespondences() const {return _number_of_measurements;}
  inline const real inlierRatio() const {assert(_number_of_measurements > 0); return static_cast<real>(_number_of_inliers)/_number_of_measurements;}
  inline const real totalError() const {return _total_error;}
  inline const real averageError() const {assert(_number_of_measurements > 0); return _total_error/_number_of_measurements;}
  inline const bool hasSystemConverged() const {return _has_system_converged;}
  inline AlignerParameters* parameters() {return _parameters;}
  void setMinimumReliableDepthMeters(const real& minimum_reliable_depth_meters_) {_minimum_reliable_depth_meters = minimum_reliable_depth_meters_;}
  void setMaximumReliableDepthMeters(const real& maximum_reliable_depth_meters_) {_maximum_reliable_depth_meters = maximum_reliable_depth_meters_;}

//ds aligner specific
protected:

  //ds control
  std::vector<real> _errors;
  std::vector<bool> _inliers;
  Count _number_of_inliers      = 0;
  Count _number_of_outliers     = 0;
  Count _number_of_measurements = 0;

  //ds linearization
  bool _has_system_converged = false;
  real _total_error          = 0;

  //ds generic thresholds (usally set by framepoint generator or tracker configuration)
  real _minimum_reliable_depth_meters = 0.01;
  real _maximum_reliable_depth_meters = 15;

  //! @brief configurable parameters
  AlignerParameters* _parameters = 0;

};

//ds class that holds all generic (running) variables used in an aligner - to be used in conjunction with an aligner
template<Count states_, Count dimension_>
class AlignerWorkspace {

//ds object handling
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds default workspace setup
  AlignerWorkspace() {}

  //ds default workspace tear-down
  ~AlignerWorkspace() {}

//ds exported types
public:

  typedef Eigen::Matrix<real, states_, states_> StateMatrix;
  typedef Eigen::Matrix<real, states_, 1> StateVector;
  typedef Eigen::Matrix<real, dimension_, 1> MeasurementVector;
  typedef Eigen::Matrix<real, dimension_, dimension_> DimensionMatrix;
  typedef Eigen::Matrix<real, dimension_, states_> JacobianMatrix;

//ds workspace variables
protected:

  StateMatrix _H                  = StateMatrix::Zero();
  StateVector _b                  = StateVector::Zero();
  DimensionMatrix _omega          = DimensionMatrix::Identity();
  JacobianMatrix _jacobian        = JacobianMatrix::Zero();
  StateMatrix _information_matrix = StateMatrix::Identity();

  std::vector<DimensionMatrix, Eigen::aligned_allocator<DimensionMatrix> > _information_matrix_vector;
  std::vector<MeasurementVector, Eigen::aligned_allocator<MeasurementVector> > _fixed;
};
}
