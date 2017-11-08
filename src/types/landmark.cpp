#include "landmark.h"

namespace proslam {

Count Landmark::_instances = 0;

Landmark::Landmark(FramePoint* origin_, const LandmarkParameters* parameters_): _identifier(_instances),
                                                                                _origin(origin_),
                                                                                _parameters(parameters_) {
  ++_instances;

  //ds allocate fresh state (trading construction correctness for enclosed generation)
  _state = new State(this, origin_->worldCoordinates());
  _states.clear();
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

void Landmark::update(const PointCoordinates& coordinates_in_world_, const real& depth_meters_) {
  assert(_state);
  assert(_parameters);

  //ds compute relative delta to the current coordinate estimate
  const real relative_delta = (_state->world_coordinates-coordinates_in_world_).norm()/depth_meters_;

  //ds check if inlier measurement or less than 2 updates yet
  if (relative_delta < _parameters->maximum_translation_error_to_depth_ratio || _number_of_updates < _parameters->minimum_number_of_forced_updates) {

    //ds current weight
    const real weight_new_measurement = 1/depth_meters_;

    //ds update total weight
    _total_weight += weight_new_measurement;

    //ds update coordinates (http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf)
    _state->world_coordinates += weight_new_measurement/_total_weight*(coordinates_in_world_-_state->world_coordinates);
    _are_coordinates_validated = true;
    ++_number_of_updates;
  } else {

    //ds discard measurement completely
    _are_coordinates_validated = false;
  }
  assert(!std::isnan(_state->world_coordinates.x()));
  assert(!std::isnan(_state->world_coordinates.y()));
  assert(!std::isnan(_state->world_coordinates.z()));
}

void Landmark::update(const FramePoint* point_) {

  //ds update appearance history
  _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorLeft()));
  _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorRight()));

  //ds always update position
  update(point_->worldCoordinates(), point_->depthMeters());
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
