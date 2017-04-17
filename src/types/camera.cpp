#include "camera.h"

namespace proslam {

//ds inner instance count
Count Camera::_instances = 0;

  //ds a camera can be constructed based on image dimensions and a camera matrix
  Camera::Camera(const Count& image_rows_,
                 const Count& image_cols_,
                 const CameraMatrix& camera_matrix_,
                 const TransformMatrix3D& offset_) {
    _identifier = _instances;
    ++_instances;

    setImageRows(image_rows_);
    setImageCols(image_cols_);
    setCameraMatrix(camera_matrix_);
    setOffset(offset_);
  }

  const bool Camera::isInFieldOfView(const PointCoordinates& image_coordinates_) const {
    assert(image_coordinates_.z() == 1);
    return ((image_coordinates_.x() >= 0 && image_coordinates_.x() <= _image_cols)&&
            (image_coordinates_.y() >= 0 && image_coordinates_.y() <= _image_rows));
  }

  void Camera::setCameraMatrix(const CameraMatrix& camera_matrix_) {
    _camera_matrix         = camera_matrix_;
    _inverse_camera_matrix = _camera_matrix.inverse();

    //ds update the projection matrix (adjusted whenever the camera matrix and/or the offset is changed)
    _projection_matrix.block<3,3>(0,0) = camera_matrix_*_robot_to_camera.linear();
    _projection_matrix.block<3,1>(0,3) = camera_matrix_*_robot_to_camera.translation();
  }

  void Camera::setOffset(const TransformMatrix3D& camera_to_robot_) {
    _camera_to_robot = camera_to_robot_;
    _robot_to_camera = _camera_to_robot.inverse();

    //ds update the projection matrix (adjusted whenever the camera matrix and/or the offset is changed)
    _projection_matrix.block<3,3>(0,0) = _camera_matrix*_robot_to_camera.linear();
    _projection_matrix.block<3,1>(0,3) = _camera_matrix*_robot_to_camera.translation();
  }
}
