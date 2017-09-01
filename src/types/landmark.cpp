#include "landmark.h"

namespace proslam {

  Count Landmark::_instances = 0;

  Landmark::Landmark(const FramePoint* origin_, const LandmarkParameters* parameters_): _identifier(_instances),
                                                                                        _origin(origin_),
                                                                                        _parameters(parameters_) {
    ++_instances;

    //ds allocate fresh state (trading construction correctness for enclosed generation)
    _state = new State(this, origin_->worldCoordinates());
  }

  Landmark::~Landmark() {

    //ds if the current state is not connected to a local map yet free it
    if (!_state->local_map) {
      delete _state;
    }
  }

  void Landmark::resetCoordinates(const PointCoordinates& coordinates_) {

    //ds update state
    _state->world_coordinates = coordinates_;
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

    //ds accumulate descriptors if relocalization is desired
    if (!_parameters->option_disable_relocalization) {
      _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorLeft()));
      _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorRight()));
    }

    //ds always update position
    update(point_->worldCoordinates(), point_->depthMeters());
  }
}
