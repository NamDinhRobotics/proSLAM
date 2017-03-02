#include "pxyz_aligner.h"

#include "types/items/gt_landmark.h"
#include "srrg_types/types.hpp"

namespace gslam {

  //ds initialize aligner with minimal entity TODO purify this
  void PXYZAligner::init(BaseContext* context_, const TransformMatrix3D& robot_to_world_) {
    _context        = static_cast<Frame*>(context_);
    _robot_to_world = robot_to_world_;
    _world_to_robot = _robot_to_world.inverse();
    _errors.resize(_context->points().size());
    _inliers.resize(_context->points().size());

    //ds unused
    _damping = 0.0;
  }

  //ds linearize the system: to be called inside oneRound
  void PXYZAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds for all the points
    for (Index index_point = 0; index_point < _context->points().size(); ++index_point) {
      _errors[index_point]  = -1;
      _inliers[index_point] = false;
      _omega.setIdentity();

      //ds buffer point
      const FramePoint* frame_point = _context->points()[index_point];

      //ds skip processing if point is not eligible
      if (frame_point->landmark() == 0 || frame_point->hasDepthByVision() || !frame_point->landmark()->isValidated()) {
        continue;
      }

      //ds compute error based on camera point
      const PointCoordinates& measured_point_in_robot = frame_point->robotCoordinates();
      const PointCoordinates& sampled_point_in_robot  = _world_to_robot*frame_point->landmark()->coordinates();
//      const gt_real sampled_depth_meters = (_context->camera()->robotToCamera()*sampled_point_in_robot).z();

      //ds 3D error
      Vector3 error = sampled_point_in_robot-measured_point_in_robot;

      //ds update chi
      const gt_real error_squared = error.transpose()*error;
      _errors[index_point] = error_squared;
//      std::cerr << measured_point_in_robot.transpose() << " | " << sampled_point_in_robot.transpose() << " error: " << _errors[index_point] <<  " depth: " << sampled_depth_meters << std::endl;

      //ds check if outlier
      if (error_squared > _maximum_error_kernel) {
        ++_number_of_outliers;
        if (ignore_outliers_) {
          continue;
        }
        _omega *=_maximum_error_kernel/error_squared;
      } else {
        _inliers[index_point] = true;
        ++_number_of_inliers;
      }
      _total_error += _errors[index_point];

      //ds get the jacobian of the transform part = [I -2*skew(T*modelPoint)]
      _jacobian.block<3,3>(0,0).setIdentity();
      _jacobian.block<3,3>(0,3) = -2*srrg_core::skew(sampled_point_in_robot);

      //ds precompute transposed
      const Matrix6_3 jacobian_transposed(_jacobian.transpose( ));

      //ds accumulate
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }
  }

  //ds solve alignment problem for one round
  void PXYZAligner::oneRound(const bool& ignore_outliers_) {

    //ds linearize system
    linearize(ignore_outliers_);
//    std::cerr << "error: " << totalError() << " inliers: " << numberOfInliers() << " outliers: " << numberOfOutliers() << std::endl;

    //ds solve the system and update the estimate
    _world_to_robot = srrg_core::v2t(static_cast<const Vector6&>(_H.ldlt().solve(-_b)))*_world_to_robot;

    //ds enforce rotation symmetry
    const Matrix3 rotation   = _world_to_robot.linear();
    Matrix3 rotation_squared = rotation.transpose( )*rotation;
    rotation_squared.diagonal().array() -= 1;
    _world_to_robot.linear() -= 0.5*rotation*rotation_squared;

    //ds update wrapped structures
    _robot_to_world = _world_to_robot.inverse();
  }

  //ds solve alignment problem until convergence is reached
  void PXYZAligner::converge() {

    //ds previous error to check for convergence
    gt_real total_error_previous = 0.0;

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
        break;
      } else {
        total_error_previous = _total_error;
      }

      //ds check last iteration
      if(iteration == _maximum_number_of_iterations-1) {
        _has_system_converged = false;
        std::cerr << "PXYZAligner::converge|WARNING: system did not converge - inlier ratio: " << static_cast<gt_real>(_number_of_inliers)/_context->points().size() << std::endl;
      }
    }
  }
} //namespace gslam
