#include "xyz_aligner.h"

#include "srrg_types/types.hpp"

namespace proslam {

  //ds initialize aligner with minimal entity TODO purify this
  void XYZAligner::init(BaseContext* context_, const TransformMatrix3D& current_to_reference_) {
    _context              = static_cast<CorrespondenceCollection*>(context_);
    _current_to_reference = current_to_reference_;

    //ds unused
    _errors.clear();
    _inliers.clear();
    _damping = 0.0;
  }

  //ds linearize the system: to be called inside oneRound
  void XYZAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds for all the points
    for (const Correspondence* correspondence: _context->correspondences) {
      _omega.setIdentity();

      //ds compute error based on items: local map merging
      const PointCoordinates& measured_point_in_reference = correspondence->item_reference->robot_coordinates;
      const PointCoordinates sampled_point_in_reference   = _current_to_reference*correspondence->item_query->robot_coordinates;
      const Vector3 error                                 = sampled_point_in_reference-measured_point_in_reference;

      //ds adjust omega to inverse depth value (the further away the point, the less weight)
      _omega(2,2) *= 1/sampled_point_in_reference.z();

      //ds update chi
      const real error_squared = error.transpose()*error;

      //ds check if outlier
      real weight = 1.0;
      if (error_squared > _maximum_error_kernel) {
        ++_number_of_outliers;
        if (ignore_outliers_) {
          continue;
        }
        weight=_maximum_error_kernel/error_squared;
      } else {
        ++_number_of_inliers;
      }
      _total_error += error_squared;

      //ds get the jacobian of the transform part = [I -2*skew(T*modelPoint)]
      _jacobian.block<3,3>(0,0).setIdentity();
      _jacobian.block<3,3>(0,3) = -2*srrg_core::skew(sampled_point_in_reference);

      //ds precompute transposed
      const Matrix6_3 jacobian_transposed(_jacobian.transpose( ));

      //ds accumulate
      _H += weight*jacobian_transposed*_omega*_jacobian;
      _b += weight*jacobian_transposed*_omega*error;
    }
  }

  //ds solve alignment problem for one round
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

  //ds solve alignment problem until convergence is reached
  void XYZAligner::converge() {

    //ds previous error to check for convergence
    real total_error_previous = 0.0;

    //ds start LS
    for (Count iteration = 0; iteration < _maximum_number_of_iterations; ++iteration) {
      oneRound(false);

      //ds check if converged (no descent required)
      if (_error_delta_for_convergence > std::fabs(total_error_previous-_total_error)) {

        //ds trigger inlier only runs
        oneRound(true);
        oneRound(true);
        oneRound(true);

        //ds system converged
        _has_system_converged = true;

        //ds compute inliers ratio
        const real inlier_ratio = static_cast<real>(_number_of_inliers)/_context->correspondences.size();

        //ds set out values
        _context->transform_frame_query_to_frame_reference = _current_to_reference;
        _context->icp_inlier_ratio         = inlier_ratio;
        _context->icp_number_of_inliers    = _number_of_inliers;
        _context->icp_number_of_iterations = iteration;

        //ds if the solution is acceptable
        if (_number_of_inliers > _minimum_number_of_inliers && inlier_ratio > _minimum_inlier_ratio) {
          std::printf( "XYZAligner::converge|found   alignment for local maps [%06lu] > [%06lu] (correspondences: %3lu, iterations: %2lu, inlier ratio: %5.3f, inliers: %2lu, kernel size: %5.3f)\n",
          _context->id_query, _context->id_reference, _context->correspondences.size( ), iteration, inlier_ratio, _number_of_inliers, _maximum_error_kernel );
          _context->is_valid = true;
          break;
        } else {
          std::printf( "XYZAligner::converge|dropped alignment for local maps [%06lu] > [%06lu] (correspondences: %3lu, iterations: %2lu, inlier ratio: %5.3f, inliers: %2lu, kernel size: %5.3f)\n",
          _context->id_query, _context->id_reference, _context->correspondences.size( ), iteration, inlier_ratio, _number_of_inliers, _maximum_error_kernel );
          _context->is_valid = false;
          break;
        }
      } else {
        total_error_previous = _total_error;
      }

      //ds check last iteration
      if(iteration == _maximum_number_of_iterations-1) {
        _has_system_converged = false;
        std::cerr << "XYZAligner::converge|WARNING: system did not converge - inlier ratio: " << static_cast<real>(_number_of_inliers)/_context->correspondences.size() << std::endl;
      }
    }
  }
}
