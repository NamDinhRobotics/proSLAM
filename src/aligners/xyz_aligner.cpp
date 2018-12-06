#include "xyz_aligner.h"

namespace proslam {

  XYZAligner::XYZAligner(AlignerParameters* parameters_): BaseLocalMapAligner(parameters_) {
    //ds nothing to do
  }

  XYZAligner::~XYZAligner() {
    //ds nothing to do
  }

  void XYZAligner::initialize(Closure* context_, const TransformMatrix3D& current_to_reference_) {

    //ds initialize base components
    _context              = context_;
    _current_to_reference = current_to_reference_;
    _parameters->damping = 0;
    _number_of_measurements = _context->correspondences.size();
    _errors.resize(_number_of_measurements);
    _inliers.resize(_number_of_measurements);

    //ds construct point cloud registration problem - compute landmark coordinates in local maps
    _information_vector.resize(_number_of_measurements);
    _moving.resize(_number_of_measurements);
    _fixed.resize(_number_of_measurements);
    const TransformMatrix3D& world_to_reference_local_map(context_->local_map_reference->worldToLocalMap());
    const TransformMatrix3D& world_to_query_local_map(context_->local_map_query->worldToLocalMap());
    for (Index u = 0; u < _number_of_measurements; ++u) {
      const Closure::Correspondence* correspondence = _context->correspondences[u];

      //ds point coordinates to register
      _fixed[u]  = world_to_reference_local_map*correspondence->reference->coordinates();
      _moving[u] = world_to_query_local_map*correspondence->query->coordinates();

      //ds set information matrix
      _information_vector[u].setIdentity();
      _information_vector[u] *= correspondence->matching_ratio;
    }
  }

  void XYZAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds for all the points
    for (Index u = 0; u < _number_of_measurements; ++u) {
      _omega = _information_vector[u];

      //ds compute error based on items: local map merging
      const PointCoordinates sampled_point_in_reference   = _current_to_reference*_moving[u];
      const Vector3 error                                 = sampled_point_in_reference-_fixed[u];

      //ds update chi
      const real error_squared = error.transpose()*_omega*error;

      //ds check if outlier
      if (error_squared > _parameters->maximum_error_kernel) {
        _inliers[u] = false;
        ++_number_of_outliers;
        if (ignore_outliers_) {
          continue;
        }

        //ds proportionally reduce information value of the measurement
        _omega *= _parameters->maximum_error_kernel/error_squared;
      } else {
        _inliers[u] = true;
        ++_number_of_inliers;
      }
      _total_error += error_squared;

      //ds get the jacobian of the transform part = [I -2*skew(T*modelPoint)]
      _jacobian.block<3,3>(0,0).setIdentity();
      _jacobian.block<3,3>(0,3) = -2*srrg_core::skew(sampled_point_in_reference);

      //ds precompute transposed
      const Matrix6_3 jacobian_transposed(_jacobian.transpose( ));

      //ds accumulate
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }
  }

  void XYZAligner::oneRound(const bool& ignore_outliers_) {

    //ds linearize system
    linearize(ignore_outliers_);

    //ds solve the system and update the estimate
    _current_to_reference = srrg_core::v2t(static_cast<const Vector6&>(_H.ldlt().solve(-_b)))*_current_to_reference;

    //ds enforce rotation symmetry
    const Matrix3 rotation   = _current_to_reference.linear();
    Matrix3 rotation_squared = rotation.transpose( )*rotation;
    rotation_squared.diagonal().array() -= 1;
    _current_to_reference.linear()      -= 0.5*rotation*rotation_squared;
  }

  void XYZAligner::converge() {

    //ds previous error to check for convergence
    real total_error_previous = 0.0;

    //ds start LS
    for (Count iteration = 0; iteration < _parameters->maximum_number_of_iterations; ++iteration) {
      oneRound(false);

      //ds check if converged (no descent required)
      if (_parameters->error_delta_for_convergence > std::fabs(total_error_previous-_total_error)) {

        //ds trigger inlier only runs
        oneRound(true);
        oneRound(true);
        oneRound(true);

        //ds system converged
        _has_system_converged = true;

        //ds compute inliers ratio
        const real inlier_ratio = static_cast<real>(_number_of_inliers)/_context->correspondences.size();

        //ds set out values
        _context->query_to_reference       = _current_to_reference;
        _context->icp_inlier_ratio         = inlier_ratio;
        _context->icp_number_of_inliers    = _number_of_inliers;
        _context->icp_number_of_iterations = iteration;

        //ds if the solution is acceptable
        if (_number_of_inliers > _parameters->minimum_number_of_inliers && inlier_ratio > _parameters->minimum_inlier_ratio) {
          LOG_INFO(std::printf("XYZAligner::converge|registered local maps [%06lu:{%06lu-%06lu}] > [%06lu:{%06lu-%06lu}] "
                               "(correspondences: %3lu, iterations: %2lu, inlier ratio: %5.3f, inliers: %2lu)\n",
          _context->local_map_query->identifier(),
          _context->local_map_query->frames().front()->identifier(), _context->local_map_query->frames().back()->identifier(),
          _context->local_map_reference->identifier(),
          _context->local_map_reference->frames().front()->identifier(), _context->local_map_reference->frames().back()->identifier(),
          _context->correspondences.size(), iteration, inlier_ratio, _number_of_inliers))

          //ds enable closure
          _context->is_valid = true;

          //ds set inlier status
          for (Index u = 0; u < _number_of_measurements; ++u) {
            _context->correspondences[u]->is_inlier = _inliers[u];
          }

          break;
        } else {
          LOG_DEBUG(std::printf("XYZAligner::converge|dropped registration for local maps [%06lu:{%06lu-%06lu}] > [%06lu:{%06lu-%06lu}] "
                               "(correspondences: %3lu, iterations: %2lu, inlier ratio: %5.3f, inliers: %2lu)\n",
          _context->local_map_query->identifier(),
          _context->local_map_query->frames().front()->identifier(), _context->local_map_query->frames().back()->identifier(),
          _context->local_map_reference->identifier(),
          _context->local_map_reference->frames().front()->identifier(), _context->local_map_reference->frames().back()->identifier(),
          _context->correspondences.size(), iteration, inlier_ratio, _number_of_inliers))
          _context->is_valid = false;
          break;
        }
      } else {
        total_error_previous = _total_error;
      }

      //ds check last iteration
      if(iteration == _parameters->maximum_number_of_iterations-1) {
        _context->is_valid = false;
        _has_system_converged = false;
        LOG_DEBUG(std::cerr << "XYZAligner::converge|system did not converge - inlier ratio: " << static_cast<real>(_number_of_inliers)/_context->correspondences.size()
                            << " [" << _context->local_map_query->identifier() << "][" << _context->local_map_reference->identifier() << "]" << std::endl)
      }
    }
  }
}
