#include "camera.h"

namespace proslam {

//ds inner instance count
Count Camera::_instances = 0;

  //ds a camera can be constructed based on image dimensions and a camera matrix
  Camera::Camera(const Count& image_rows_,
                 const Count& image_cols_,
                 const CameraMatrix& camera_matrix_,
                 const TransformMatrix3D& camera_to_robot_): _identifier(_instances),
                                                             _image_rows(image_rows_),
                                                             _image_cols(image_cols_) {
    ++_instances;
    setCameraMatrix(camera_matrix_);
    setCameraToRobot(camera_to_robot_);
  }

  const bool Camera::isInFieldOfView(const PointCoordinates& image_coordinates_) const {
    assert(image_coordinates_.z() == 1);
    return ((image_coordinates_.x() >= 0 && image_coordinates_.x() <= _image_cols)&&
            (image_coordinates_.y() >= 0 && image_coordinates_.y() <= _image_rows));
  }

  void Camera::setCameraMatrix(const CameraMatrix& camera_matrix_) {
    _camera_matrix         = camera_matrix_;
    _inverse_camera_matrix = _camera_matrix.inverse();
  }

  void Camera::setCameraToRobot(const TransformMatrix3D& camera_to_robot_) {
    _camera_to_robot = camera_to_robot_;
    _robot_to_camera = _camera_to_robot.inverse();
  }
}
