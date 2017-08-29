#include "landmark.h"

namespace proslam {

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  Count Landmark::_instances = 0;

  //ds initial landmark coordinates must be provided
  Landmark::Landmark(const FramePoint* origin_, const LandmarkParameters* parameters_): _identifier(_instances),
                                                                                        _origin(origin_),
                                                                                        _parameters(parameters_) {
    ++_instances;

    //ds allocate fresh state (trading construction correctness for enclosed generation)
    _state = new State(this, origin_->worldCoordinates());
  }

  //ds cleanup of dynamic structures
  Landmark::~Landmark() {

    //ds if the current state is not connected to a local map yet - the landmark has to clean it up
    if (!_state->local_map) {
      delete _state;
    }
  }

  //ds reset landmark coordinates to a certain position (loss of past measurements!)
  void Landmark::resetCoordinates(const PointCoordinates& coordinates_, const real& weight_) {

    //ds clear past measurements
    _total_weight         = weight_;
    _number_of_updates    = 0;
    _number_of_recoveries = 0;

    //ds add fresh measurement
    update(coordinates_);
  }

  //ds landmark coordinates update - no visual information (e.g. map optimization)
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

  //ds landmark coordinates update with visual information (tracking)
  void Landmark::update(const FramePoint* point_) {

    //ds always update descriptors
    _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorLeft()));
    _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), point_->descriptorRight()));

    //ds update position
    update(point_->worldCoordinates(), point_->depthMeters());
  }
}
