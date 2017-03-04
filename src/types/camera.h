#pragma once
#include "definitions.h"

namespace gslam {

  class Camera {

  //ds object handling
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Camera(const Count& image_rows_,
           const Count& image_cols_,
           const CameraMatrix& camera_matrix_,
           const TransformMatrix3D& offset_  = TransformMatrix3D::Identity());

  public:

    inline const Identifier index() const {return _index;}
    inline const Count imageRows() const {return _image_rows;}
    inline const Count imageCols() const {return _image_cols;}
    const bool isInFieldOfView(const PointCoordinates& image_coordinates_) const;
    void setImageRows(const Count& image_rows_) {_image_rows = image_rows_;}
    void setImageCols(const Count& image_cols_) {_image_cols = image_cols_;}
    inline const CameraMatrix& cameraMatrix() const {return _camera_matrix;}
    inline const CameraMatrix& inverseCameraMatrix() const {return _inverse_camera_matrix;}
    void setCameraMatrix(const CameraMatrix& camera_matrix_);
    inline const ProjectionMatrix& projectionMatrix() const {return _projection_matrix;}
    inline const TransformMatrix3D& cameraToRobot() const {return _camera_to_robot;}
    inline const TransformMatrix3D& robotToCamera() const {return _robot_to_camera;}
    void setOffset(const TransformMatrix3D& camera_to_robot_);
    inline const gt_real depthConversionFactor() const {return _depth_conversion_factor;}
    void setDepthConversionFactor(const gt_real& depth_conversion_factor_) {_depth_conversion_factor = depth_conversion_factor_;}

  protected:

    Identifier _index = 0;
    CameraMatrix  _camera_matrix        = CameraMatrix::Zero();
    CameraMatrix _inverse_camera_matrix = CameraMatrix::Zero();
    ProjectionMatrix _projection_matrix = ProjectionMatrix::Zero();

    Count _image_rows = 0;
    Count _image_cols = 0;
    gt_real _depth_conversion_factor  = 0.0;
    TransformMatrix3D _camera_to_robot         = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_camera = TransformMatrix3D::Identity();

  private:

    static Identifier _instances;

  };

  typedef std::vector<Camera*> CameraPtrVector;
  typedef std::map<std::string, Camera*> StringCameraMap;
  typedef std::pair<std::string, Camera*> StringCameraMapElement;
}
