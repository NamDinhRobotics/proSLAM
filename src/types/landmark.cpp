#include "landmark.h"
#include "local_map.h"

namespace proslam {

Count Landmark::_instances = 0;

Landmark::Landmark(FramePoint* origin_, const LandmarkParameters* parameters_): _identifier(_instances),
                                                                                _origin(origin_),
                                                                                _parameters(parameters_) {
  ++_instances;
  _measurements.clear();
  _appearances.clear();
  _states_in_local_maps.clear();

  //ds compute initial position estimate as rude average of the track
  //ds we do not weight the measurements with disparity/inverse depth here
  //ds since invalid, small depths can lead to a fatal initial guess
  _world_coordinates.setZero();
  FramePoint* framepoint = origin_;
  while (framepoint) {
    _measurements.push_back(Measurement(framepoint));
    _origin = framepoint;
    _world_coordinates += framepoint->worldCoordinates();
    framepoint = framepoint->previous();
  }
  _world_coordinates /= _measurements.size();
  _number_of_updates = _measurements.size();
}

Landmark::~Landmark() {
  _appearances.clear();
  _measurements.clear();
  _states_in_local_maps.clear();
}

void Landmark::addState(State* landmark_state_) {
  _states_in_local_maps.push_back(landmark_state_);
  _appearances.clear();
}

void Landmark::update(const FramePoint* point_) {
  if (!point_) {
    return;
  }

  //ds update appearance history (left descriptors only)
  _appearances.push_back(point_->descriptorLeft());
  _measurements.push_back(Measurement(point_));

  //ds trigger classic ICP in camera update of landmark coordinates - setup
  Vector3 world_coordinates(_world_coordinates);
  Matrix3 H(Matrix3::Zero());
  Vector3 b(Vector3::Zero());
  Matrix3 jacobian;
  Matrix3 jacobian_transposed;
  Matrix3 omega(Matrix3::Identity());
  real total_error_squared_previous = 0;
  const real maximum_error_squared_meters = 5*5;

  //ds gauss newton descent
  for (uint32_t iteration = 0; iteration < 1000; ++iteration) {
    H.setZero();
    b.setZero();
    real total_error_squared    = 0;
    uint32_t number_of_outliers = 0;

    //ds for each measurement
    for (const Measurement& measurement: _measurements) {
      omega.setIdentity();

      //ds sample current state in measurement context
      const PointCoordinates camera_coordinates_sampled = measurement.world_to_camera*world_coordinates;
      if (camera_coordinates_sampled.z() <= 0) {
        ++number_of_outliers;
        continue;
      }

      //ds compute error
      const Vector3 error(camera_coordinates_sampled-measurement.camera_coordinates);

      //ds weight inverse depth
      omega *= measurement.inverse_depth_meters;

      //ds update chi
      const real error_squared = error.transpose()*omega*error;
      total_error_squared += error_squared;

      //ds robust kernel
      if (error_squared > maximum_error_squared_meters) {
        omega *= maximum_error_squared_meters/error_squared;
        ++number_of_outliers;
      }

      //ds get the jacobian of the transform part: R
      jacobian = measurement.world_to_camera.linear();

      //ds precompute transposed
      jacobian_transposed = jacobian.transpose();

      //ds accumulate
      H += jacobian_transposed*omega*jacobian;
      b += jacobian_transposed*omega*error;
    }

    //ds update state
    world_coordinates += H.fullPivLu().solve(-b);

    //ds check convergence
    if (std::fabs(total_error_squared-total_error_squared_previous) < 1e-5 || iteration == 999) {
      const uint32_t number_of_inliers = _measurements.size()-number_of_outliers;

      //ds if the number of inliers is higher than the best so far
      if (number_of_inliers > _number_of_updates) {

        //ds update landmark state
        _world_coordinates = world_coordinates;
        _number_of_updates        = number_of_inliers;

      //ds if optimization failed and we have less inliers than outliers - reset initial guess
      } else if (number_of_inliers < number_of_outliers) {

        //ds reset estimate based on overall average
        PointCoordinates world_coordinates_accumulated(PointCoordinates::Zero());
        for (const Measurement& measurement: _measurements) {
          world_coordinates_accumulated += measurement.world_coordinates;
        }

        //ds set landmark state without increasing update count
        _world_coordinates = world_coordinates_accumulated/_measurements.size();
      }
      break;
    }

    //ds update previous
    total_error_squared_previous = total_error_squared;
  }
}

void Landmark::merge(Landmark* landmark_) {
  assert(landmark_ != this);

  //ds update recent appearances
  _appearances.insert(_appearances.end(), landmark_->_appearances.begin(), landmark_->_appearances.end());
  landmark_->_appearances.clear();

  //ds update states of absorbed landmark and add them to this states
  for (State* state: landmark_->_states_in_local_maps) {
    for (HBSTMatchable* appearance: state->appearances) {
      appearance->objects.begin()->second = this;
    }
    state->landmark = this;
  }
  _states_in_local_maps.insert(_states_in_local_maps.end(), landmark_->_states_in_local_maps.begin(), landmark_->_states_in_local_maps.end());
  landmark_->_states_in_local_maps.clear();

  //ds compute new merged world coordinates
  _world_coordinates = (_number_of_updates*_world_coordinates+
                       landmark_->_number_of_updates*landmark_->_world_coordinates)
                       /(_number_of_updates+landmark_->_number_of_updates);

  //ds update measurements
  _number_of_updates    += landmark_->_number_of_updates;
  _number_of_recoveries += landmark_->_number_of_recoveries;
  _measurements.insert(_measurements.end(), landmark_->_measurements.begin(), landmark_->_measurements.end());
  landmark_->_measurements.clear();

  //ds update past framepoint pointers
  FramePoint* framepoint = landmark_->_origin;
  while (framepoint) {
    framepoint->setLandmark(this);
    framepoint = framepoint->next();
  }
}
}
