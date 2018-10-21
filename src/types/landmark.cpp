#include "landmark.h"

namespace proslam {

Count Landmark::_instances = 0;

Landmark::Landmark(FramePoint* origin_, const LandmarkParameters* parameters_): _identifier(_instances),
                                                                                _origin(origin_),
                                                                                _parameters(parameters_) {
  ++_instances;
  _measurements.clear();

  //ds compute initial position estimate as rude average of the track
  //ds we do not weight the measurements with disparity/inverse depth here
  //ds since invalid, small depths can lead to a fatal initial guess
  PointCoordinates world_coordinates(PointCoordinates::Zero());
  FramePoint* framepoint = origin_;
  while (framepoint) {
    _measurements.push_back(Measurement(framepoint));
    _origin = framepoint;
    world_coordinates += framepoint->worldCoordinates();
    framepoint = framepoint->previous();
  }
  world_coordinates /= _measurements.size();
  _number_of_updates = _measurements.size();

  //ds allocate fresh state (trading construction correctness for enclosed generation)
  _states.clear();
  _state = new State(this, world_coordinates);
}

Landmark::~Landmark() {

  //ds if the current state is not connected to a local map yet free it
  if (!_state->local_map) {
    delete _state;
  }
  _states.clear();
}

void Landmark::renewState() {

  //ds bookkeep the old state
  _states.push_back(_state);

  //ds allocate a new state
  _state = new State(this, _state->world_coordinates);
}

void Landmark::update(const FramePoint* point_) {

  //ds update appearance history
  _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorLeft()));
  _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorRight()));
  _measurements.push_back(Measurement(point_));

  //ds trigger classic ICP in camera update of landmark coordinates - setup
  Vector3 world_coordinates(_state->world_coordinates);
  Matrix3 H(Matrix3::Zero());
  Vector3 b(Vector3::Zero());
  Matrix3 jacobian;
  Matrix3 jacobian_transposed;
  Matrix3 omega(Matrix3::Identity());
  real total_error_squared_previous = 0;
  const real maximum_error_squared_meters = 5*5;

  //ds gauss newton descent
  //std::cerr << "[ ] x = " << world_coordinates.transpose() << std::endl;
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
      //std::cerr << camera_coordinates_sampled.transpose() << " - " << camera_coordinates_measured.transpose() << " = " << error.transpose() << std::endl;

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
//    std::cerr << "[" << u << "] x = " << world_coordinates.transpose()
//                          << " average error: " << total_error_squared/_measurements.size()
//                          << " outliers: " << number_of_outliers << "/" << _measurements.size() <<  std::endl;

    //ds check convergence
    if (std::fabs(total_error_squared-total_error_squared_previous) < 1e-5 || iteration == 999) {
      const uint32_t number_of_inliers = _measurements.size()-number_of_outliers;

      //ds if the number of inliers is higher than the best so far
      if (number_of_inliers > _number_of_updates) {

        //ds update landmark state
        _state->world_coordinates = world_coordinates;
        _number_of_updates        = number_of_inliers;

      //ds if optimization failed and we have less inliers than outliers - reset initial guess
      } else if (number_of_inliers < number_of_outliers) {

        //ds reset estimate based on overall average
        PointCoordinates world_coordinates_accumulated(PointCoordinates::Zero());
        for (const Measurement& measurement: _measurements) {
          world_coordinates_accumulated += measurement.world_coordinates;
        }

        //ds set landmark state without increasing update count
        _state->world_coordinates = world_coordinates_accumulated/_measurements.size();
      }
      break;
    }

    //ds update previous
    total_error_squared_previous = total_error_squared;
  }
}

void Landmark::merge(Landmark* landmark_) {
  assert(landmark_ != this);

  //ds update active state
  _state->appearances.insert(_state->appearances.end(), landmark_->_state->appearances.begin(), landmark_->_state->appearances.end());
  _state->coordinates_in_local_map = landmark_->_state->coordinates_in_local_map;
  _state->world_coordinates        = (_number_of_updates*_state->world_coordinates+
                                      landmark_->_number_of_updates*landmark_->_state->world_coordinates)
                                     /(_number_of_updates+landmark_->_number_of_updates);

  //ds update information filter
  _total_weight         += landmark_->_total_weight;
  _number_of_updates    += landmark_->_number_of_updates;
  _number_of_recoveries += landmark_->_number_of_recoveries;

  //ds update past framepoint pointers
  FramePoint* framepoint = landmark_->_origin;
  while (framepoint) {
    framepoint->setLandmark(this);
    framepoint = framepoint->next();
  }

  //ds update pointer in the linked local maps
  landmark_->_state->landmark = this;
  for (State* state: landmark_->_states) {
    state->landmark = this;
  }

  //ds merge states from input landmark
  _states.insert(_states.end(), landmark_->_states.begin(), landmark_->_states.end());
}
}
