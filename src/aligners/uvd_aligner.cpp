#include "uvd_aligner.h"
#include "types/landmark.h"

namespace proslam {
  using namespace srrg_core;

  UVDAligner::UVDAligner(AlignerParameters* parameters_): BaseFrameAligner(parameters_) {}
  UVDAligner::~UVDAligner() {}

  //ds initialize aligner with minimal entity
  void UVDAligner::initialize(const Frame* frame_previous_,
                              const Frame* frame_current_,
                              const TransformMatrix3D& previous_to_current_) {
    _frame_previous      = frame_previous_;
    _frame_current       = frame_current_;
    _previous_to_current = previous_to_current_;

    //ds prepare buffers
    _number_of_measurements = _frame_current->points().size();
    _errors.resize(_number_of_measurements);
    _inliers.resize(_number_of_measurements);
    _information_matrix_vector.resize(_number_of_measurements);
    _weights_translation.resize(_number_of_measurements, 1);
    _moving.resize(_number_of_measurements);
    _fixed.resize(_number_of_measurements);

    //ds fill buffers
    for (Index u = 0; u < _number_of_measurements; ++u) {
      const FramePoint* frame_point = _frame_current->points()[u];
      _information_matrix_vector[u].setIdentity();

      //ds measurement: set fixed part U, V, Depth
      _fixed[u](0) = frame_point->imageCoordinatesLeft().x();
      _fixed[u](1) = frame_point->imageCoordinatesLeft().y();
      _fixed[u](2) = frame_point->cameraCoordinatesLeft().z();

      //ds if we have a landmark
      if (frame_point->landmark()) {

        //ds prefer landmark estimate
        _moving[u] = frame_point->previous()->cameraCoordinatesLeftLandmark();

        //ds increase weight linear in the number of updates
        _information_matrix_vector[u] *= (1+frame_point->landmark()->numberOfUpdates());
      } else {

        //ds set moving part (3D point coordinates)
        _moving[u] = frame_point->previous()->cameraCoordinatesLeft();
      }

      //ds we scale the depth error by a factor of 10 to be competitive with the U V (pixel) error which is integer
      _information_matrix_vector[u](2,2) = 10.0;

      //ds if we cannot consider the translation contribution of the point
      if (frame_point->hasEstimatedDepth()) {

        //ds block translation contribution
        _weights_translation[u] = 0;

        //ds no error on the depth
        _information_matrix_vector[u](2,2) = 0;
      }
    }

    //ds wrappers for optimization
    _camera_calibration_matrix = _frame_current->cameraLeft()->cameraMatrix();
    _number_of_rows_image      = _frame_current->cameraLeft()->numberOfImageRows();
    _number_of_cols_image      = _frame_current->cameraLeft()->numberOfImageCols();
  }

  //ds linearize the system: to be called inside oneRound
  void UVDAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers  = 0;
    _number_of_outliers = 0;
    _total_error        = 0;

    //ds loop over all points (assumed to have previous points)
    for (Index u = 0; u < _frame_current->points().size(); ++u) {
      _errors[u]  = -1;
      _inliers[u] = false;
      _omega      = _information_matrix_vector[u];

      //ds compute the point in the camera frame
      const PointCoordinates predicted_point_in_camera = _previous_to_current*_moving[u];
      const real depth_meters                          = predicted_point_in_camera.z();
      if (depth_meters <= _minimum_depth) {
        continue;
      }

      //ds retrieve homogeneous projections
      const PointCoordinates predicted_uvd_in_camera = _camera_calibration_matrix*predicted_point_in_camera;
      
      //ds compute the image coordinates (homogeneous division)
      PointCoordinates predicted_point_in_image  = predicted_uvd_in_camera/predicted_uvd_in_camera.z();

      //ds restore the depth in the third component as we will confront it with the measured depth from the sensor
      predicted_point_in_image.z() = depth_meters;
      
      //ds if the point is outside the image, skip
      if (predicted_point_in_image.x() < 0 || predicted_point_in_image.x() > _number_of_cols_image||
          predicted_point_in_image.y() < 0 || predicted_point_in_image.y() > _number_of_rows_image) {
        continue;
      }

      //ds compute error (U V D)
      const Vector3 error(predicted_point_in_image.x()-_fixed[u](0),
                          predicted_point_in_image.y()-_fixed[u](1),
                          predicted_point_in_image.z()-_fixed[u](2));

      //ds compute squared error
      const real chi = error.transpose()*_omega*error;

      //ds update error stats
      _errors[u] = chi;

      //ds check if outlier
      if (chi > _parameters->maximum_error_kernel) {
        if (ignore_outliers_) {
          continue;
        }

        //ds proportionally reduce information value of the measurement
        _omega *= _parameters->maximum_error_kernel/chi;
      } else {
        _inliers[u] = true;
        ++_number_of_inliers;
      }

      //ds update total error
      _total_error += _errors[u];

      //ds precompute partial derivatives of homogeneous division
      const real inverse_z         = 1/depth_meters;
      const real inverse_z_squared = inverse_z*inverse_z;

      //ds compute the jacobian of the transformation
      Matrix3_6 jacobian_transform;

      //ds translation contribution
      jacobian_transform.block<3,3>(0,0) = _weights_translation[u]*Matrix3::Identity();

      //ds always consider rotation
      jacobian_transform.block<3,3>(0,3) = -2*skew(predicted_point_in_camera);

      //ds jacobian parts of the homogeneous division (note that we have a 3x3 instead of a 2x3 matrix since we want to map the depth as well)
      Matrix3 jacobian_projection;
      jacobian_projection << inverse_z, 0, -predicted_uvd_in_camera.x()*inverse_z_squared,
                             0, inverse_z, -predicted_uvd_in_camera.y()*inverse_z_squared,
                             0, 0, 1;

      //ds assemble final jacobian
      _jacobian = jacobian_projection*_camera_calibration_matrix*jacobian_transform;

      //ds precompute transposed
      const Matrix6_3 jacobian_transposed(_jacobian.transpose());

      //ds update H and b
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }
    _number_of_outliers = _number_of_measurements-_number_of_inliers;
  }

  //ds solve alignment problem for one round
  void UVDAligner::oneRound(const bool& ignore_outliers_) {

    //ds linearize system once
    linearize(ignore_outliers_);

    //ds damping
    _H += _parameters->damping*_number_of_measurements*Matrix6::Identity();

    //ds compute dense LS solution transformation after perturbation
    const Vector6 perturbation = _H.fullPivLu().solve(-_b);
    _previous_to_current       = v2t(perturbation)*_previous_to_current;

    //ds enforce proper rotation matrix
    const Matrix3 rotation               = _previous_to_current.linear();
    Matrix3 rotation_squared             = rotation.transpose() * rotation;
    rotation_squared.diagonal().array() -= 1;
    _previous_to_current.linear()       -= 0.5*rotation*rotation_squared;
  }

  //ds solve alignment problem until convergence is reached
  void UVDAligner::converge() {

    //ds previous error to check for convergence
    real total_error_previous = 0;

    //ds start LS
    for (Count iteration = 0; iteration < _parameters->maximum_number_of_iterations; ++iteration) {
      oneRound(false);

      //ds check if converged (no descent required)
      if (_parameters->error_delta_for_convergence > std::fabs(total_error_previous-_total_error)) {
        total_error_previous = _total_error;

        //ds if we have at least a certain number of inliers and more inliers than outliers - trigger inlier only runs
        if (_number_of_inliers > 100 && _number_of_inliers > _number_of_outliers) {
          for (Count iteration_inlier = 0; iteration_inlier < _parameters->maximum_number_of_iterations; ++iteration_inlier) {
            oneRound(true);

            //ds check for convergence
            if (std::fabs(total_error_previous-_total_error) < _parameters->error_delta_for_convergence) {
              total_error_previous = _total_error;
              break;
            } else {
              total_error_previous = _total_error;
            }
          }
        }

        //ds compute information matrix
        _information_matrix = _H;

        //ds system converged
        _has_system_converged = true;
        break;
      } else {
        total_error_previous = _total_error;
      }

      //ds check last iteration
      if(iteration == _parameters->maximum_number_of_iterations-1) {
        _has_system_converged = false;
        LOG_WARNING(std::cerr << "UVDAligner::converge|system did not converge - total error: "  << _total_error
                              << " average error: " << _total_error/(_number_of_inliers+_number_of_outliers)
                              << " inliers: " << _number_of_inliers << " outliers: " << _number_of_outliers << std::endl)
      }
    }

    //ds VISUALIZATION ONLY
    for (Index u = 0; u < _number_of_measurements; ++u) {
      FramePoint* frame_point = _frame_current->points()[u];
      ImageCoordinates image_coordinates(_camera_calibration_matrix*_previous_to_current*_moving[u]);
      image_coordinates /= image_coordinates.z();
      frame_point->setProjectionEstimateLeftOptimized(cv::Point2f(image_coordinates.x(), image_coordinates.y()));
    }
  }
}
