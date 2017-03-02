#include "uvd_aligner.h"

#include "srrg_types/types.hpp"
#include "types/items/gt_landmark.h"

namespace gslam {

  //ds initialize aligner with minimal entity (TODO purify this, currently Frame)
  void UVDAligner::init(BaseContext* context_, const TransformMatrix3D& initial_guess_robot_to_world_) {
    _context            = static_cast<Frame*>(context_);
    _errors.resize(_context->points().size());
    _inliers.resize(_context->points().size());
    _robot_pose         = _context->robotToWorld();
    _inverse_robot_pose = _context->worldToRobot();
  }

  //ds linearize the system: to be called inside oneRound
  void UVDAligner::linearize(const bool& ignore_outliers_) {
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;
    const TransformMatrix3D& robot_to_camera = _context->camera()->robotToCamera();
    const Matrix3& camera_matrix             = _context->camera()->cameraMatrix();
    const Count image_rows                   = _context->camera()->imageRows();
    const Count image_cols                   = _context->camera()->imageCols();

    //ds loop over all points
    for (Index index_point = 0; index_point < _context->points().size(); index_point++) {
      _errors[index_point]  = -1;
      _inliers[index_point] = false;
      _omega.setIdentity();

      //gg get the data
      FramePoint* frame_point = _context->points()[index_point];
      Landmark* landmark      = frame_point->landmark();

      //ds skip point without or with invalid landmark
      if (!landmark) {
        continue;
      }

      if (!landmark->isValidated()) {
        continue;
      }

      const PointCoordinates& measured_point_in_image = frame_point->imageCoordinates();
      const PointCoordinates& measured_world_point    = landmark->coordinates();
      assert(_context->camera()->isInFieldOfView(measured_point_in_image));

      //gg compute the point in the robot frame and in the camera frame TODO verify depth correctness
      const PointCoordinates sampled_point_in_robot  = _inverse_robot_pose*measured_world_point;
      const PointCoordinates sampled_point_in_camera = robot_to_camera*sampled_point_in_robot;

      //gg if the point is behind or in the camera, skip
      if (sampled_point_in_camera.z() <= 0) {
        continue;
      }

      //ds compute projection
      const PointCoordinates sampled_abc = camera_matrix*sampled_point_in_camera;
      const gt_real sampled_c            = sampled_abc.z();

      //gg compute the image coordinates
      const gt_real inverse_sampled_c               = 1/sampled_c;
      const PointCoordinates sampled_point_in_image = sampled_abc/sampled_c;

      //gg if the point is outside the image, skip
      if (sampled_point_in_image.x() < 0 || sampled_point_in_image.x() > image_cols||
          sampled_point_in_image.y() < 0 || sampled_point_in_image.y() > image_rows) {
        continue;
      }
      assert(_context->camera()->isInFieldOfView(sampled_point_in_image));

      //ds precompute
      const gt_real inverse_sampled_c_squared = inverse_sampled_c*inverse_sampled_c;
      frame_point->setReprojectionCoordinates(sampled_point_in_image);

      //gg compute the jacobian of the transformation
      Matrix3_6 Jt;
      Jt.block<3,3>(0,0).setIdentity();
      Jt.block<3,3>(0,3)=-srrg_core::skew(sampled_point_in_robot);

      //gg compute the jacobian of the projection
      Matrix3 Jp;
      Jp <<
      inverse_sampled_c, 0,  -sampled_abc.x()*inverse_sampled_c_squared,
      0,  inverse_sampled_c, -sampled_abc.y()*inverse_sampled_c_squared,
      0,  0,  1;

      _jacobian = Jp*camera_matrix*robot_to_camera.linear()*Jt;

      Vector3 point_error = sampled_point_in_image-measured_point_in_image;
      assert(point_error.z() == 0);
      //point_error.z() = sampled_point_in_camera.z()-frame_point->depth();
      if (frame_point->hasDepth()) {
        //_omega(2,2) = _weight_depth*inverse_sampled_depth;
        _omega *= (1+_weight_depth/sampled_point_in_camera.z());
      } else if (frame_point->hasDepthByVision()) {
        //_omega(2,2) = _weight_depth*inverse_sampled_depth_squared;
        _omega *= (1+_weight_depth/(sampled_point_in_camera.z()*sampled_point_in_camera.z()));
      } else {
        point_error.z() = 0;
      }

      //ds compute squared error
      const gt_real chi = point_error.transpose()*point_error;

      //ds update error stats
      _errors[index_point] = chi;

      //ds if outlier
      if (chi > _maximum_error_kernel) {
        _number_of_outliers++;
        if (ignore_outliers_) {
          continue;
        }
        _omega *= _maximum_error_kernel/chi;
      } else {
        _inliers[index_point] = true;
        _number_of_inliers++;
      }

      //ds update total error
      _total_error += chi;
      landmark->setIsActive(true);

      //ds update H and b
      _H += _jacobian.transpose()*_omega*_jacobian;
      _b += _jacobian.transpose()*_omega*point_error;
    }
  }

  //ds solve alignment problem for one round
  void UVDAligner::oneRound(const bool& ignore_outliers_) {
    _H.setZero();
    _b.setZero();
    linearize(ignore_outliers_);

    _H += _damping*BaseAligner6_3::AlignmentMatrix::Identity();
    BaseAligner6_3::AlignmentVector dx = _H.ldlt().solve(-_b);

    _inverse_robot_pose = srrg_core::v2tEuler(dx)*_inverse_robot_pose;
    BaseAligner6_3::InformationMatrix R = _inverse_robot_pose.linear();
    BaseAligner6_3::InformationMatrix E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _inverse_robot_pose.linear() -= 0.5 * R * E;
    _robot_pose=_inverse_robot_pose.inverse();
  }

  void UVDAligner::converge() {
    //ds TODO implement
  }
} //namespace gslam
