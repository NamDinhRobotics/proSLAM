#include "landmark.h"

namespace proslam {

  Identifier Landmark::_instances = 0;

  Landmark::Landmark(const PointCoordinates& point_coordinates_): _index(_instances),
                                                                  _coordinates(point_coordinates_) {
    ++_instances;
    _is_validated      = false;
    _is_close          = true;
    _is_active         = false;
    _first_observation = 0;
    _measurements_test.clear();

    //ds allocate fresh item (trading construction correctness for enclosed generation)
    _item = new Item(this);
  }

  Landmark::~Landmark() {
    if (_item->local_map == 0) {
      delete _item;
    }
    _measurements_test.clear();
  }

  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const real& depth_meters_) {

    //ds if we got at least 2 previous measurements
    if (_measurements_test.size() > 1) {

      //ds compute average delta
      const real relative_delta = (_coordinates_average_previous-coordinates_in_world_).norm()/depth_meters_;

      //ds if inlier measurement
      if (relative_delta < 0.5) {

        //ds update average for next measurement addition
        _coordinates_average_previous = (_measurements_test.size()*_coordinates_average_previous+coordinates_in_world_)/(_measurements_test.size()+1);

        //ds update coordinates
        _measurements_test.push_back(std::make_pair(1/depth_meters_, coordinates_in_world_));
        PointCoordinates coordinates_final(PointCoordinates::Zero());
        real total_weight = 0;
        for (const std::pair<real, PointCoordinates> measurement: _measurements_test) {
          total_weight      += measurement.first;
          coordinates_final += measurement.first*measurement.second;
        }
        _coordinates = coordinates_final/total_weight;
        _is_validated = true;
      } else {

        //ds discard measurement completely TODO add kernel?
        _is_validated = false;
        ++_number_of_failed_updates;
      }
    } else {

      //ds update coordinates based on average
      _measurements_test.push_back(std::make_pair(1/depth_meters_, coordinates_in_world_));
      PointCoordinates coordinates_average(PointCoordinates::Zero());
      for (const std::pair<real, PointCoordinates> measurement: _measurements_test) {
        coordinates_average += measurement.second;
      }
      _coordinates                  = coordinates_average/_measurements_test.size();
      _coordinates_average_previous = _coordinates;
      _is_validated                 = true;
    }
    assert(!std::isnan(_coordinates.x()));
    assert(!std::isnan(_coordinates.y()));
    assert(!std::isnan(_coordinates.z()));
    ++_number_of_updates;
  }

  void Landmark::update(const PointCoordinates& coordinates_in_world_,
                        const cv::Mat& descriptor_left_,
                        const cv::Mat& descriptor_right_,
                        const real& depth_meters_) {

    //ds always update descriptors
    _item->appearances.push_back(new Appearance(_item, descriptor_left_));
    _item->appearances.push_back(new Appearance(_item, descriptor_right_));

    //ds update position
    update(coordinates_in_world_, depth_meters_);
  }

  void Landmark::resetCoordinates(const PointCoordinates& coordinates_) {

    //ds clear past measurements
    _measurements_test.clear();
    _number_of_failed_updates = 0;
    _number_of_updates        = 0;

    //ds add fresh measurement
    update(coordinates_);
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
