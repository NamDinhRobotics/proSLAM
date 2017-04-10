#include "landmark.h"

namespace proslam {

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  Count Landmark::_instances = 0;

  //ds initial landmark coordinates must be provided
  Landmark::Landmark(const FramePoint* origin_): _identifier(_instances),
                                                 _origin(origin_),
                                                 _coordinates(origin_->worldCoordinates()) {
    ++_instances;

    //ds allocate fresh state (trading construction correctness for enclosed generation)
    _state = new State(this);
  }

  //ds cleanup of dynamic structures
  Landmark::~Landmark() {

    //ds if the current state is not connected to a local map yet - we have to clean it up
    if (_state->local_map == 0) {
      delete _state;
    }
  }

  //ds reset landmark coordinates to a certain position (loss of past measurements!)
  void Landmark::resetCoordinates(const PointCoordinates& coordinates_) {

    //ds clear past measurements
    _total_weight = 0;
    _number_of_updates = 0;

    //ds add fresh measurement
    update(coordinates_);
  }

  //ds landmark coordinates update - no visual information (e.g. map optimization)
  void Landmark::update(const PointCoordinates& coordinates_in_world_, const real& depth_meters_) {

    //ds compute relative delta to the current coordinate estimate
    const real relative_delta = (_coordinates-coordinates_in_world_).norm()/depth_meters_;

    //ds check if inlier measurement
    if (relative_delta < 0.5 || _number_of_updates < 2) {

      //ds current weight
      const real weight_new_measurement = 1/depth_meters_;

      //ds update total weight
      _total_weight += weight_new_measurement;

      //ds update coordinates (http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf)
      _coordinates += weight_new_measurement/_total_weight*(coordinates_in_world_-_coordinates);
      _are_coordinates_validated = true;
      ++_number_of_updates;
    } else {

      //ds discard measurement completely
      _are_coordinates_validated = false;
    }
    assert(!std::isnan(_coordinates.x()));
    assert(!std::isnan(_coordinates.y()));
    assert(!std::isnan(_coordinates.z()));
  }

  //ds landmark coordinates update with visual information (tracking)
  void Landmark::update(const FramePoint* point_) {

    //ds always update descriptors
    _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), getDescriptor(point_->descriptorLeft())));
    _state->appearances.push_back(new HBSTMatchable(reinterpret_cast<const void*>(_state), getDescriptor(point_->descriptorRight())));

    //ds update position
    update(point_->worldCoordinates(), point_->depthMeters());
  }

  Landmark* LandmarkPointerMap::get(const Identifier& identifier_) {
    LandmarkPointerMap::iterator iterator = find(identifier_);
    if (iterator == end()) {
      return 0;
    } else {
      return iterator->second;
    }
  }

  void LandmarkPointerMap::put(Landmark* landmark) {
    assert(find(landmark->identifier()) == end());
    insert(std::make_pair(landmark->identifier(), landmark));
  }
}
