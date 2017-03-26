#include "landmark.h"

namespace proslam {

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  Count Landmark::_instances = 0;

  //ds initial landmark coordinates must be provided
  Landmark::Landmark(const PointCoordinates& point_coordinates_): _identifier(_instances),
                                                                  _coordinates(point_coordinates_) {
    ++_instances;
    _updates.clear();

    //ds allocate fresh state (trading construction correctness for enclosed generation)
    _state = new State(this);
  }

  //ds cleanup of dynamic structures
  Landmark::~Landmark() {

    //ds if the current state is not connected to a local map yet - we have to clean it up
    if (_state->local_map == 0) {
      delete _state;
    }

    //ds clear measurements
    _updates.clear();
  }

  //ds reset landmark coordinates to a certain position (loss of past measurements!)
  void Landmark::resetCoordinates(const PointCoordinates& coordinates_) {

    //ds clear past measurements
    _updates.clear();

    //ds add fresh measurement
    update(coordinates_);
  }

  //ds landmark coordinates update - no visual information (e.g. map optimization)
  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const real& depth_meters_) {

    //ds if we got at least 2 previous measurements
    if (_updates.size() > 1) {

      //ds compute average delta
      const real relative_delta = (_coordinates_average_previous-coordinates_in_world_).norm()/depth_meters_;

      //ds if inlier measurement
      if (relative_delta < 0.5) {

        //ds update average for next measurement addition
        _coordinates_average_previous = (_updates.size()*_coordinates_average_previous+coordinates_in_world_)/(_updates.size()+1);

        //ds update coordinates
        _updates.push_back(std::make_pair(1/depth_meters_, coordinates_in_world_));
        PointCoordinates coordinates_final(PointCoordinates::Zero());
        real total_weight = 0;
        for (const std::pair<real, PointCoordinates> measurement: _updates) {
          total_weight      += measurement.first;
          coordinates_final += measurement.first*measurement.second;
        }
        _coordinates = coordinates_final/total_weight;
        _are_coordinates_validated = true;
      } else {

        //ds discard measurement completely
        _are_coordinates_validated = false;
      }
    } else {

      //ds update coordinates based on average
      _updates.push_back(std::make_pair(1/depth_meters_, coordinates_in_world_));
      PointCoordinates coordinates_average(PointCoordinates::Zero());
      for (const std::pair<real, PointCoordinates> measurement: _updates) {
        coordinates_average += measurement.second;
      }
      _coordinates                  = coordinates_average/_updates.size();
      _coordinates_average_previous = _coordinates;
      _are_coordinates_validated    = true;
    }
    assert(!std::isnan(_coordinates.x()));
    assert(!std::isnan(_coordinates.y()));
    assert(!std::isnan(_coordinates.z()));
  }

  //ds landmark coordinates update with visual information (tracking)
  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const cv::Mat& descriptor_left_,
                        const cv::Mat& descriptor_right_,
                        const real& depth_meters_) {

    //ds always update descriptors
    _state->appearances.push_back(new Appearance(_state, descriptor_left_));
    _state->appearances.push_back(new Appearance(_state, descriptor_right_));

    //ds update position
    update(coordinates_in_world_, depth_meters_);
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

  //ds visualization only
  void LandmarkPointerMap::clearActive() {
    for (iterator iterator = begin(); iterator != end(); ++iterator) {
      iterator->second->setIsActive(false);
    }
  }
}
