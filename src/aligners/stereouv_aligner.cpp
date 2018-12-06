#include "stereouv_aligner.h"
#include "types/landmark.h"

namespace proslam {
  using namespace srrg_core;

  StereoUVAligner::StereoUVAligner(AlignerParameters* parameters_): BaseFrameAligner(parameters_) {}

  StereoUVAligner::~StereoUVAligner() {}

  //ds initialize aligner with minimal entity
  void StereoUVAligner::initialize(const Frame* frame_previous_,
                                   const Frame* frame_current_,
                                   const TransformMatrix3D& previous_to_current_) {
    _frame_previous      = frame_previous_;
    _frame_current       = frame_current_;
    _previous_to_current = previous_to_current_;

    //ds prepare buffers
    _number_of_measurements = _frame_current->points().size();
    _errors.resize(_number_of_measurements);
    _inliers.resize(_number_of_measurements);
    _information_vector.resize(_number_of_measurements);
    _weights_translation.resize(_number_of_measurements, 1);
    _moving.resize(_number_of_measurements);
    _fixed.resize(_number_of_measurements);

    //ds fill buffers
    for (Index u = 0; u < _number_of_measurements; ++u) {
      const FramePoint* frame_point = _frame_current->points()[u];
      _information_vector[u].setIdentity();

      assert(_frame_current->cameraLeft()->isInFieldOfView(frame_point->imageCoordinatesLeft()));
      assert(_frame_current->cameraRight()->isInFieldOfView(frame_point->imageCoordinatesRight()));
      assert(frame_point->previous());

      //ds set fixed part (image coordinates)
      _fixed[u](0) = frame_point->imageCoordinatesLeft().x();
      _fixed[u](1) = frame_point->imageCoordinatesLeft().y();
      _fixed[u](2) = frame_point->imageCoordinatesRight().x();
      _fixed[u](3) = frame_point->imageCoordinatesRight().y();

      //ds if we have a landmark
      if (frame_point->landmark()) {

        //ds prefer landmark estimate
        _moving[u] = frame_point->previous()->cameraCoordinatesLeftLandmark();

        //ds increase weight linear in the number of updates
        _information_vector[u] *= (1+frame_point->landmark()->numberOfUpdates());
      } else {

        //ds set moving part (3D point coordinates)
        _moving[u] = frame_point->previous()->cameraCoordinatesLeft();
      }

      //ds scale information proportional to disparity of the measurement (the bigger, the closer, the better the triangulation)
      _information_vector[u] *= std::log(1+frame_point->disparityPixels())/(1+std::fabs(frame_point->epipolarOffset()));
    }

    //ds if individual weighting is desired
    if (_enable_weights_translation) {
      for (Index u = 0; u < _number_of_measurements; ++u) {
        _weights_translation[u] = _maximum_reliable_depth_meters/_moving[u].z();
      }
    }

    //ds wrappers for optimization
    _camera_calibration_matrix = _frame_current->cameraLeft()->cameraMatrix();
    _offset_camera_right       = _frame_current->cameraRight()->baselineHomogeneous();
    _number_of_rows_image      = _frame_current->cameraLeft()->numberOfImageRows();
    _number_of_cols_image      = _frame_current->cameraLeft()->numberOfImageCols();
  }

  //ds linearize the system: to be called inside oneRound
  void StereoUVAligner::linearize(const bool& ignore_outliers_) {

    //ds initialize setup
    _H.setZero();
    _b.setZero();
    _number_of_inliers = 0;
    _total_error       = 0;

    //ds loop over all current framepoints (assuming that each of them has a previous one)
    for (Index u = 0; u < _number_of_measurements; ++u) {
      _errors[u]  = -1;
      _inliers[u] = false;
      _omega      = _information_vector[u];

      //ds compute the point in the camera frame - prefering a landmark estimate if available
      const PointCoordinates sampled_point_in_camera_left = _previous_to_current*_moving[u];
      if (sampled_point_in_camera_left.z() <= _minimum_depth) {
        continue;
      }

      //ds retrieve homogeneous projections
      const Vector4 sampled_point_in_camera_left_homogeneous(sampled_point_in_camera_left.x(),
                                                             sampled_point_in_camera_left.y(),
                                                             sampled_point_in_camera_left.z(),
                                                             1);
      const PointCoordinates sampled_abc_in_camera_left  = _camera_calibration_matrix*sampled_point_in_camera_left;
      const PointCoordinates sampled_abc_in_camera_right = sampled_abc_in_camera_left+_offset_camera_right;
      const real& sampled_c_left  = sampled_abc_in_camera_left.z();
      const real& sampled_c_right = sampled_abc_in_camera_right.z();

      //ds compute the image coordinates
      const PointCoordinates sampled_point_in_image_left  = sampled_abc_in_camera_left/sampled_c_left;
      const PointCoordinates sampled_point_in_image_right = sampled_abc_in_camera_right/sampled_c_right;

      //ds if the point is outside the image, skip
      if (sampled_point_in_image_left.x() < 0 || sampled_point_in_image_left.x() > _number_of_cols_image||
          sampled_point_in_image_left.y() < 0 || sampled_point_in_image_left.y() > _number_of_rows_image) {
        continue;
      }
      if (sampled_point_in_image_right.x() < 0 || sampled_point_in_image_right.x() > _number_of_cols_image||
          sampled_point_in_image_right.y() < 0 || sampled_point_in_image_right.y() > _number_of_rows_image) {
        continue;
      }
      assert(_frame_current->cameraLeft()->isInFieldOfView(sampled_point_in_image_left));
      assert(_frame_current->cameraRight()->isInFieldOfView(sampled_point_in_image_right));

      //ds compute error (we compute the vertical error only once, since we assume rectified cameras)
      const Vector4 error(sampled_point_in_image_left.x()-_fixed[u](0),
                          sampled_point_in_image_left.y()-_fixed[u](1),
                          sampled_point_in_image_right.x()-_fixed[u](2),
                          sampled_point_in_image_right.y()-_fixed[u](3));

      //ds weight all measurements proportional to their distance to the sensor (squared to damp the effect)
      //const real weight_translation = 1/std::sqrt(depth_meters);
      //const real weight_translation = _maximum_reliable_depth_meters/depth_meters;
      //const real weight_translation = exp(-depth_meters/_maximum_reliable_depth_meters);
      //const real weight_translation = 1; //std::sqrt(frame_point->disparityPixels());

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

      //ds compute the jacobian of the transformation
      Matrix3_6 jacobian_transform;

      //ds translation contribution (will be scaled with omega)
//      const real translation_weight = _maximum_reliable_depth_meters/sampled_point_in_camera_left.z();
      jacobian_transform.block<3,3>(0,0) = _weights_translation[u]*Matrix3::Identity();

      //ds rotation contribution - compensate for inverse depth (far points should have an equally strong contribution as close ones)
      jacobian_transform.block<3,3>(0,3) = -2*skew(sampled_point_in_camera_left);

      //ds precompute
      const Matrix3_6 camera_matrix_per_jacobian_transform(_camera_calibration_matrix*jacobian_transform);

      //ds precompute
      const real inverse_sampled_c_left  = 1/sampled_c_left;
      const real inverse_sampled_c_right = 1/sampled_c_right;
      const real inverse_sampled_c_squared_left  = inverse_sampled_c_left*inverse_sampled_c_left;
      const real inverse_sampled_c_squared_right = inverse_sampled_c_right*inverse_sampled_c_right;

      //ds jacobian parts of the homogeneous division: left
      Matrix2_3 jacobian_left;
      jacobian_left << inverse_sampled_c_left, 0, -sampled_abc_in_camera_left.x()*inverse_sampled_c_squared_left,
                       0, inverse_sampled_c_left, -sampled_abc_in_camera_left.y()*inverse_sampled_c_squared_left;

      //ds we compute only the contribution for the horizontal error: right
      Matrix2_3 jacobian_right;
      jacobian_right << inverse_sampled_c_right, 0, -sampled_abc_in_camera_right.x()*inverse_sampled_c_squared_right,
                        0, inverse_sampled_c_right, -sampled_abc_in_camera_right.y()*inverse_sampled_c_squared_right;

      //ds the last rows of jacobian_left and jacobian_right are identical for perfect, horizontally triangulated points
      //ds in a horizontal stereo camera configuration

      //ds assemble final jacobian
      _jacobian.setZero();

      //ds we have to compute the full block
      _jacobian.block<2,6>(0,0) = jacobian_left*camera_matrix_per_jacobian_transform;

      //ds we only have to compute the horizontal block
      _jacobian.block<2,6>(2,0) = jacobian_right*camera_matrix_per_jacobian_transform;

      //ds precompute transposed
      const Matrix6_4 jacobian_transposed(_jacobian.transpose());

      //ds update H and b
      _H += jacobian_transposed*_omega*_jacobian;
      _b += jacobian_transposed*_omega*error;
    }

    //ds update statistics
    _number_of_outliers = _number_of_measurements-_number_of_inliers;
  }

  //ds solve alignment problem for one round
  void StereoUVAligner::oneRound(const bool& ignore_outliers_) {

    //ds linearize system once
    linearize(ignore_outliers_);

    //ds damping
    _H += _parameters->damping*_number_of_measurements*Matrix6::Identity();

    //ds compute solution transformation after perturbation
    const Vector6 dx     = _H.fullPivLu().solve(-_b);
    _previous_to_current = v2t(dx)*_previous_to_current;

    //ds enforce proper rotation matrix
    const Matrix3 rotation               = _previous_to_current.linear();
    Matrix3 rotation_squared             = rotation.transpose() * rotation;
    rotation_squared.diagonal().array() -= 1;
    _previous_to_current.linear()       -= 0.5*rotation*rotation_squared;
  }

  //ds solve alignment problem until convergence is reached
  void StereoUVAligner::converge() {

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
        LOG_WARNING(std::cerr << "StereoUVAligner::converge|system did not converge - total error: "  << _total_error
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
