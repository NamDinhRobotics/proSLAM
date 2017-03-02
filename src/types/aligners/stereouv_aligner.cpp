#include "stereouv_aligner.h"

#include "srrg_types/types.hpp"
#include "types/items/gt_landmark.h"
#include "types/stereo_grid_detector.h"

namespace gslam {

  using namespace srrg_core;

  //ds initialize aligner with minimal entity (TODO purify this, currently Frame)
  void StereoUVAligner::init(BaseContext* context_, const TransformMatrix3D& robot_to_world_) {
    _context = static_cast<Frame*>(context_);
    _errors.resize(_context->points().size());
    _inliers.resize(_context->points().size());
    _robot_to_world = robot_to_world_;
    _world_to_robot = _robot_to_world.inverse();

    //ds wrappers for optimization
    _camera_left_to_world = _robot_to_world*_context->camera()->cameraToRobot();
    _world_to_camera_left = _camera_left_to_world.inverse();
    _projection_matrix_left  = _context->camera()->projectionMatrix();
    _projection_matrix_right = _context->cameraExtra()->projectionMatrix();
    _image_rows = _context->camera()->imageRows();
    _image_cols = _context->camera()->imageCols();

    //ds others
    _robot_to_world_previous = _context->previous()->robotToWorld();
  }

  //ds linearize the system: to be called inside oneRound
  void StereoUVAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds loop over all points
    for (Index index_point = 0; index_point < _context->points().size(); index_point++) {
      _errors[index_point]  = -1;
      _inliers[index_point] = false;
      _omega.setIdentity();

      //ds buffer references
      FramePoint* frame_point  = _context->points()[index_point];
      const Landmark* landmark = frame_point->landmark();

      assert(_context->camera()->isInFieldOfView(frame_point->imageCoordinates()));
      assert(_context->cameraExtra()->isInFieldOfView(frame_point->imageCoordinatesExtra()));

      //ds compute the point in the camera frame
      PointCoordinates sampled_point_in_camera_left = PointCoordinates::Zero();
      if (landmark && landmark->isValidated()) {
        sampled_point_in_camera_left = _world_to_camera_left*landmark->coordinates();
      } else {
        sampled_point_in_camera_left = _world_to_camera_left*(_robot_to_world_previous*frame_point->previous()->robotCoordinates());
        _omega *= _weight_framepoint;
      }
      const gt_real& depth_meters = sampled_point_in_camera_left.z();
      if (depth_meters <= 0 || depth_meters > StereoGridDetector::maximum_depth_far) {
        continue;
      }

      //ds retrieve homogeneous projections
      const Vector4 sampled_point_in_camera_left_homogeneous(sampled_point_in_camera_left.x(), sampled_point_in_camera_left.y(), sampled_point_in_camera_left.z(), 1);
      const PointCoordinates sampled_abc_in_camera_left  = _projection_matrix_left*sampled_point_in_camera_left_homogeneous;
      const PointCoordinates sampled_abc_in_camera_right = _projection_matrix_right*sampled_point_in_camera_left_homogeneous;
      const gt_real& sampled_c_left  = sampled_abc_in_camera_left.z();
      const gt_real& sampled_c_right = sampled_abc_in_camera_right.z();

      //ds compute the image coordinates
      const PointCoordinates sampled_point_in_image_left  = sampled_abc_in_camera_left/sampled_c_left;
      const PointCoordinates sampled_point_in_image_right = sampled_abc_in_camera_right/sampled_c_right;

      //ds if the point is outside the image, skip
      if (sampled_point_in_image_left.x() < 0 || sampled_point_in_image_left.x() > _image_cols||
          sampled_point_in_image_left.y() < 0 || sampled_point_in_image_left.y() > _image_rows) {
        continue;
      }
      if (sampled_point_in_image_right.x() < 0 || sampled_point_in_image_right.x() > _image_cols||
          sampled_point_in_image_right.y() < 0 || sampled_point_in_image_right.y() > _image_rows) {
        continue;
      }
      assert(_context->camera()->isInFieldOfView(sampled_point_in_image_left));
      assert(_context->cameraExtra()->isInFieldOfView(sampled_point_in_image_right));

      //ds precompute
      const gt_real inverse_sampled_c_left  = 1/sampled_c_left;
      const gt_real inverse_sampled_c_right = 1/sampled_c_right;
      const gt_real inverse_sampled_c_squared_left  = inverse_sampled_c_left*inverse_sampled_c_left;
      const gt_real inverse_sampled_c_squared_right = inverse_sampled_c_right*inverse_sampled_c_right;
      frame_point->setReprojectionCoordinates(sampled_point_in_image_left);
      frame_point->setReprojectionCoordinatesExtra(sampled_point_in_image_right);

      //ds compute error
      Vector4 error(sampled_point_in_image_left.x()-frame_point->imageCoordinates().x(),
                    sampled_point_in_image_left.y()-frame_point->imageCoordinates().y(),
                    sampled_point_in_image_right.x()-frame_point->imageCoordinatesExtra().x(),
                    sampled_point_in_image_right.y()-frame_point->imageCoordinatesExtra().y());
      assert(error(1) == error(3));

      //ds compute squared error
      const gt_real chi = error.transpose()*error;

      //ds update error stats
      _errors[index_point] = chi;

      //ds if outlier
      if (chi > _maximum_error_kernel) {
        ++_number_of_outliers;
        if (ignore_outliers_) {
          continue;
        }

        //ds encode weight in omega
        _omega *= _maximum_error_kernel/chi;
      } else {
        _inliers[index_point] = true;
        ++_number_of_inliers;
      }

      //ds update total error
      _total_error += _errors[index_point];

      //ds compute the jacobian of the transformation
      Matrix4_6 jacobian_transform;
      jacobian_transform.setZero();

      //ds if the point is near enough - we consider translation
      if (depth_meters < StereoGridDetector::maximum_depth_close) {
        jacobian_transform.block<3,3>(0,0).setIdentity();
      }

      //ds always consider rotation
      jacobian_transform.block<3,3>(0,3) = -2*skew(sampled_point_in_camera_left);

      //ds jacobian parts of the homogeneous division
      Matrix2_3 jacobian_left;
      jacobian_left << inverse_sampled_c_left, 0, -sampled_abc_in_camera_left.x()*inverse_sampled_c_squared_left,
                       0, inverse_sampled_c_left, -sampled_abc_in_camera_left.y()*inverse_sampled_c_squared_left;
      Matrix2_3 jacobian_right;
      jacobian_right << inverse_sampled_c_right, 0, -sampled_abc_in_camera_right.x()*inverse_sampled_c_squared_right,
                        0, inverse_sampled_c_right, -sampled_abc_in_camera_right.y()*inverse_sampled_c_squared_right;

      //ds assemble final jacobian
      _jacobian.setZero();
      _jacobian.block<2,6>(0,0) = jacobian_left*_projection_matrix_left*jacobian_transform;
      _jacobian.block<2,6>(2,0) = jacobian_right*_projection_matrix_right*jacobian_transform;

      //ds precompute transposed
      const Matrix6_4 jacobian_transposed(_jacobian.transpose());

      if (depth_meters < StereoGridDetector::maximum_depth_close) {
        _omega *= (StereoGridDetector::maximum_depth_close-depth_meters)/StereoGridDetector::maximum_depth_close;
      } else {
        _omega *= (StereoGridDetector::maximum_depth_far-depth_meters)/StereoGridDetector::maximum_depth_far;
      }

      //ds update H and b
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }
  }

  //ds solve alignment problem for one round
  void StereoUVAligner::oneRound(const bool& ignore_outliers_) {

    //ds linearize system once
    linearize(ignore_outliers_);

    //ds always damp
    _H += _damping*Matrix6::Identity();

    //ds compute solution transformation
    const Vector6 dx = _H.ldlt().solve(-_b);
    _world_to_camera_left = v2t(dx)*_world_to_camera_left;

    //ds enforce rotation matrix
    const Matrix3 R = _world_to_camera_left.linear();
    Matrix3 E       = R.transpose() * R;
    E.diagonal().array()     -= 1;
    _world_to_camera_left.linear() -= 0.5 * R * E;
    _camera_left_to_world = _world_to_camera_left.inverse();

    //ds update wrapped structures
    _robot_to_world = _camera_left_to_world*_context->camera()->robotToCamera();
    _world_to_robot = _robot_to_world.inverse();
  }

  //ds solve alignment problem until convergence is reached
  void StereoUVAligner::converge() {

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

        //ds compute information matrix
        _information_matrix = _H;

        //ds system converged
        _has_system_converged = true;
        break;
      } else {
        total_error_previous = _total_error;
      }

      //ds check last iteration
      if(iteration == _maximum_number_of_iterations-1) {
        _has_system_converged = false;
        std::cerr << "StereoUVAligner::converge|WARNING: system did not converge - total error: "  << _total_error
                  << " average error: " << _total_error/(_number_of_inliers+_number_of_outliers)
                  << " inliers: " << _number_of_inliers << " outliers: " << _number_of_outliers << std::endl;
      }
    }
  }
} //namespace gslam
