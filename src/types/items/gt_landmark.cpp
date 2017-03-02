#include "gt_landmark.h"

#include "gt_landmark_item.h"
#include "types/stereo_grid_detector.h"

namespace gslam {

  Identifier Landmark::_instances = 0;

  Landmark::Landmark(const PointCoordinates& point_coordinates_): _index(_instances),
                                                                  _coordinates(point_coordinates_) {
    ++_instances;
    _is_validated      = false;
    _information_vector.setZero();
    _information_matrix.setZero();
    _is_by_vision      = false;
    _is_active         = false;
    _first_observation = 0;
    _measurements.clear();
  }

  Landmark::~Landmark() {

    //ds clear appearances
    for (const Appearance* appearance: _descriptor_track) {
      delete appearance;
    }
    _descriptor_track.clear();

    //ds clear measurements
    if (_measurements.size() > 0) {
      for (const PointCoordinatesMeasurement* measurement: _measurements) {
        delete measurement;
      }
      _measurements.clear();
    }

    //ds TODO o mei
  }

  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const Matrix3& information_,
                        const cv::Mat& descriptor_) {
    assert(_current_item != 0);

    //ds update position
    updateCoordinates(coordinates_in_world_, information_);

    //ds always update item TODO check performance if only updated for valid update
    _current_item->addSpatials(_coordinates);
    _current_item->addAppearance(descriptor_);
    ++_number_of_updates;
  }

  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const PointCoordinates& coordinates_in_robot_,
                        const TransformMatrix3D& world_to_camera_left_,
                        const cv::Mat& descriptor_left_,
                        const cv::Mat& descriptor_right_,
                        const gt_real& depth_meters_,
                        const FramePoint* frame_point_) {
    assert(_current_item != 0);

    //ds add measurement
    _measurements.push_back(new PointCoordinatesMeasurement(coordinates_in_world_, coordinates_in_robot_, world_to_camera_left_, depth_meters_));
    assert(_current_item != 0);
    assert(!std::isnan(coordinates_in_world_.x()));
    assert(!std::isnan(coordinates_in_world_.y()));
    assert(!std::isnan(coordinates_in_world_.z()));

    //ds initial values
    Matrix3 H(Matrix3::Zero());
    Vector3 b(Vector3::Zero());
    Matrix3 omega(Matrix3::Identity());
    gt_real total_error_previous = 0;
    const gt_real maximum_error_kernel = 0.1;
    _is_validated = false;

    //ds 3d point to optimize - brute average prior
    PointCoordinates coordinates_in_world_sampled(PointCoordinates::Zero());
    for (const PointCoordinatesMeasurement* measurement: _measurements) {
      coordinates_in_world_sampled += measurement->coordinates_in_world;
    }
    coordinates_in_world_sampled /= _measurements.size();

    //ds if we have enough measurements to start an optimization - at least 3
    if (_measurements.size() > 2) {

      //ds iterations (break-out if convergence reached early)
      for (uint32_t iteration = 0; iteration < 100; ++iteration) {

        //ds counts
        gt_real total_error_current = 0;
        Count number_of_inliers     = 0;

        //ds initialize setup
        H.setZero();
        b.setZero();

        //ds do calibration over all recorded values
        for (const PointCoordinatesMeasurement* measurement: _measurements) {
          omega.setIdentity();

          //ds get error
          const Vector3 error(coordinates_in_world_sampled-measurement->coordinates_in_world);
          const gt_real depth_meters((measurement->world_to_camera_left*coordinates_in_world_sampled).z());
          if (depth_meters <= 0 || depth_meters > StereoGridDetector::maximum_depth_far) {
            continue;
          }

          //ds current error
          const gt_real error_squared = error.transpose()*error;

          //ds check if outlier
          const gt_real maximum_error_kernel_relative = depth_meters*maximum_error_kernel;
          if (error_squared > maximum_error_kernel_relative) {
            omega *= maximum_error_kernel_relative/error_squared;
          } else {
            ++number_of_inliers;
          }
          total_error_current += error_squared;

          //ds aggressive information value
          omega *= 1/depth_meters;

          //ds accumulate (special case as jacobian is the identity)
          H += omega;
          b += omega*error;
        }

        //ds update x solution
        coordinates_in_world_sampled += H.ldlt().solve(-b);

        //ds check if we have converged
        if (std::fabs(total_error_previous-total_error_current) < 1e-5) {

          //ds compute inliers ratio
          _inlier_ratio = static_cast<gt_real>(number_of_inliers)/_measurements.size();

          //ds if by vision - less restrictive optimization requirements
          if (isByVision()) {
//            if (_inlier_ratio > 0) {
              _coordinates = coordinates_in_world_sampled;
              _is_validated = true;
//            } else {
//              ++_number_of_failed_updates;
//            }
          } else {
            if (_inlier_ratio > 0.3) {
              _coordinates = coordinates_in_world_sampled;
              _is_validated = true;
            } else {
              ++_number_of_failed_updates;
            }
          }
          break;
        } else {

          //ds update error
          total_error_previous = total_error_current;
        }
      }
    } else {
      _coordinates  = coordinates_in_world_sampled;
      _is_validated = true;
    }
    assert(!std::isnan(_coordinates.x()));
    assert(!std::isnan(_coordinates.y()));
    assert(!std::isnan(_coordinates.z()));

    //ds always update item TODO check performance if only updated for valid update
    _current_item->addSpatials(_coordinates);
    _current_item->addAppearance(descriptor_left_);
    _current_item->addAppearance(descriptor_right_);
    ++_number_of_updates;
  }

  void Landmark::updateCoordinates(const PointCoordinates& coordinates_in_world_,
                                   const Matrix3& information_) {

    //ds update position optimization
    _information_matrix += information_;
    _information_vector += information_*coordinates_in_world_;
    _eigensolver.compute(_information_matrix);
    assert(!std::isnan(_eigensolver.eigenvalues()(0)));
    assert(!std::isnan(_eigensolver.eigenvalues()(2)));

    //ds check result quality
    if (_eigensolver.eigenvalues()(0)/_eigensolver.eigenvalues()(2) > _min_eigenratio) {
      _coordinates  = _information_matrix.inverse()*_information_vector;
      _is_validated = true;
    } else {
      _is_validated = false;
    }
  }

  void Landmark::resetCoordinates() {
    _information_vector.setZero();
    _information_matrix.setZero();
    _number_of_updates = 0;

    //ds clear measurements
    if (_measurements.size() > 0) {
      for (const PointCoordinatesMeasurement* measurement: _measurements) {
        delete measurement;
      }
      _measurements.clear();
    }
  }

  void Landmark::createNewItem(const PointCoordinates& spatials_in_world_) {
    _current_item = new LandmarkItem(this, spatials_in_world_);
  }

  void Landmark::releaseItem() {
    _descriptor_track.insert(_descriptor_track.end(), _current_item->appearances().begin(), _current_item->appearances().end());
    createNewItem(_current_item->spatials());
  }

  Landmark* LandmarkPtrMap::get(int index) {
    LandmarkPtrMap::iterator it=find(index);
    if (it==end())
      return 0;
    return it->second;
  }

  void LandmarkPtrMap::put(Landmark* landmark) {
    LandmarkPtrMap::iterator it=find(landmark->index());
    if (it!=end())
      throw std::runtime_error("LandmarkPtrMap::put(...), double insertion");
    insert(std::make_pair(landmark->index(), landmark));
  }

  void LandmarkPtrMap::clearActive() {
    for (iterator it=begin(); it!=end(); it++)
      it->second->setIsActive(false);
  }

}
