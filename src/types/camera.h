#pragma once
#include "definitions.h"

namespace proslam {

  //ds pinhole camera object
  class Camera {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds a camera can be constructed based on image dimensions and a camera matrix
    Camera(const Count& image_rows_,
           const Count& image_cols_,
           const CameraMatrix& camera_matrix_,
           const TransformMatrix3D& camera_to_robot_ = TransformMatrix3D::Identity());

  //ds getters/setters
  public:

    inline const Identifier identifier() const {return _identifier;}
    inline const Count imageRows() const {return _image_rows;}
    inline const Count imageCols() const {return _image_cols;}
    const bool isInFieldOfView(const PointCoordinates& image_coordinates_) const;
    void setImageRows(const Count& image_rows_) {_image_rows = image_rows_;}
    void setImageCols(const Count& image_cols_) {_image_cols = image_cols_;}
    inline const CameraMatrix& cameraMatrix() const {return _camera_matrix;}
    void setCameraMatrix(const CameraMatrix& camera_matrix_);
    inline const ProjectionMatrix& projectionMatrix() const {return _projection_matrix;}
    void setProjectionMatrix(const ProjectionMatrix& projection_matrix_) {_projection_matrix = projection_matrix_;}
    inline const TransformMatrix3D& cameraToRobot() const {return _camera_to_robot;}
    inline const TransformMatrix3D& robotToCamera() const {return _robot_to_camera;}
    void setCameraToRobot(const TransformMatrix3D& camera_to_robot_);
    inline const Matrix3 rectificationMatrix() const {return _rectification_matrix;}
    void setRectificationMatrix(const Matrix3& rectification_matrix_) {_rectification_matrix = rectification_matrix_;}
    inline const Vector5 distortionCoefficients() const {return _distortion_coefficients;}
    void setDistortionCoefficients(const Vector5& distortion_coefficients_) {_distortion_coefficients = distortion_coefficients_;}

  //ds attributes
  protected:

    Identifier _identifier;
    Count _image_rows;
    Count _image_cols;
    CameraMatrix  _camera_matrix        = CameraMatrix::Zero();
    CameraMatrix _inverse_camera_matrix = CameraMatrix::Zero();
    ProjectionMatrix _projection_matrix = ProjectionMatrix::Zero();
    Matrix3 _rectification_matrix       = Matrix3::Zero();
    Vector5 _distortion_coefficients    = Vector5::Zero();

    //ds configuration on robot
    TransformMatrix3D _camera_to_robot = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_camera = TransformMatrix3D::Identity();

  //ds class specific
  private:

    //ds inner instance count
    static Count _instances;

  };

  typedef std::vector<Camera*> CameraPointerVector;
  typedef std::map<std::string, Camera*> CameraMap;
  typedef std::pair<std::string, Camera*> CameraMapElement;
}
