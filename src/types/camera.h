#pragma once
#include "definitions.h"

namespace proslam {

//! @class single pinhole model based camera object
class Camera {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//ds object handling
public:

  //! @constructor  a camera can be constructed based on image dimensions and a camera matrix
  //! @param[in] number_of_image_rows_ number of rows of the image (pixel height)
  //! @param[in] number_of_image_cols_ number of columns of the image (pixel width)
  //! @param[in] camera_matrix_ pinhole model camera matrix
  //! @param[in] camera_to_robot_ camera to robot transform (usually constant during operation)
  Camera(const Count& number_of_image_rows_,
         const Count& number_of_image_cols_,
         const CameraMatrix& camera_matrix_,
         const TransformMatrix3D& camera_to_robot_ = TransformMatrix3D::Identity());

  //! @brief prohibit default construction
  Camera() = delete;

  //! @brief default destructor
  ~Camera() {};

//ds getters/setters
public:

  inline const Identifier& identifier() const {return _identifier;}
  inline const Count& numberOfImageRows() const {return _number_of_image_rows;}
  inline const Count& numberOfImageCols() const {return _number_of_image_cols;}

  //! @brief check whether an image point is contained in the current image dimensions or not
  //! @param[in] image_coordinates_ 2D pixel coordinates of the point to evaluate
  //! @return true: if image_coordinates_ lies in the image plane, false: otherwise
  const bool isInFieldOfView(const ImageCoordinates& image_coordinates_) const;

  void setNumberOfImageRows(const Count& number_of_image_rows_) {_number_of_image_rows = number_of_image_rows_;}
  void setNumberOfImageCols(const Count& number_of_image_cols_) {_number_of_image_cols = number_of_image_cols_;}
  inline const CameraMatrix& cameraMatrix() const {return _camera_matrix;}
  void setCameraMatrix(const CameraMatrix& camera_matrix_);
  inline const ProjectionMatrix& projectionMatrix() const {return _projection_matrix;}
  void setProjectionMatrix(const ProjectionMatrix& projection_matrix_) {_projection_matrix = projection_matrix_;}
  inline const Vector3& baselineHomogeneous() const {return _baseline_homogeneous;}
  void setBaselineHomogeneous(const Vector3& baseline_homogeneous_) {_baseline_homogeneous = baseline_homogeneous_;}
  inline const TransformMatrix3D& cameraToRobot() const {return _camera_to_robot;}
  inline const TransformMatrix3D& robotToCamera() const {return _robot_to_camera;}
  void setCameraToRobot(const TransformMatrix3D& camera_to_robot_);
  inline const Matrix3& rectificationMatrix() const {return _rectification_matrix;}
  void setRectificationMatrix(const Matrix3& rectification_matrix_) {_rectification_matrix = rectification_matrix_;}
  inline const Vector5& distortionCoefficients() const {return _distortion_coefficients;}
  void setDistortionCoefficients(const Vector5& distortion_coefficients_) {_distortion_coefficients = distortion_coefficients_;}

  //! @brief write object configuration to stream
  void writeConfiguration(std::ostream& stream_) const;

//ds attributes
protected:

  //! @brief numerical identifier
  Identifier _identifier;

  //! @brief number of rows of the image (pixel height)
  Count _number_of_image_rows;

  //! @brief number of columns of the image (pixel width)
  Count _number_of_image_cols;

  //! @brief pinhole model camera matrix
  CameraMatrix _camera_matrix;

  //! @brief inverse of the pinhole model camera matrix
  CameraMatrix _inverse_camera_matrix;

  //! @brief stereo projection matrix (must be set manually with setProjectionMatrix) TODO be removed, instead use baseline
  ProjectionMatrix _projection_matrix = ProjectionMatrix::Zero();

  //! @brief stereo camera basline in homogeneous coordinates
  Vector3 _baseline_homogeneous = Vector3::Zero();

  //! @brief rectification matrix
  Matrix3 _rectification_matrix = Matrix3::Zero();

  //! @brief distortion coefficients
  Vector5 _distortion_coefficients = Vector5::Zero();

  //! @brief camera to robot transform (usually constant during operation)
  TransformMatrix3D _camera_to_robot;

  //! @brief robot to camera transform (usually constant during operation)
  TransformMatrix3D _robot_to_camera;

//ds class specific
private:

  //! @brief object instance count (used for identifier generation)
  static Count _instances;
};
}
