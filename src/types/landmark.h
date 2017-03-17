#pragma once
#include "appearance.h"

namespace proslam {

  class LandmarkItem;
  class PoseItemCollection;
  class Landmark {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    protected:

      //ds owned by world map
      Landmark(const PointCoordinates& point_coordinates_);
      ~Landmark();
      Landmark() = delete;

    public:

      //ds point with index
      inline const Index index() const {return _index;}
      inline const PointCoordinates& coordinates() const { return _coordinates; }
      inline void setCoordinates(const PointCoordinates& coordinates_) {_coordinates = coordinates_;}

      inline const AppearancePtrVector& descriptor_track() const {return _descriptor_track;}
      inline const bool isValidated() const {return _is_validated;}
      inline const bool isActive() const {return _is_active; }
      inline void setIsActive(const bool& is_active) {_is_active = is_active;}
      inline const bool isOptimized() const {return _is_optimized; }
      inline void setIsOptimized(const bool& is_optimized_) {_is_optimized = is_optimized_;}
      inline const bool isByVision() const {return _is_by_vision;}
      inline void setIsByVision(const bool& is_by_vision_) { _is_by_vision = is_by_vision_;}
      inline const bool isContained() const {return _is_contained;}
      inline void setIsContained(const bool& is_contained_) {_is_contained = is_contained_;}
      inline const bool isInLoopClosureQuery() const {return _is_in_loop_closure_query;}
      inline const bool isInLoopClosureReference() const {return _is_in_loop_closure_reference;}
      inline void setIsInLoopClosureQuery(const bool& is_in_loop_closure_query_) {_is_in_loop_closure_query = is_in_loop_closure_query_;}
      inline void setIsInLoopClosureReference(const bool& is_in_loop_closure_reference_) {_is_in_loop_closure_reference = is_in_loop_closure_reference_;}
      inline void setIsClosed(const bool& is_closed_) {_is_closed = is_closed_;}
      inline const bool isClosed() const {return _is_closed;}

      inline FramePoint* firstObservation() const {return _first_observation;}
      inline void firstObservation(FramePoint* first_observation) { _first_observation=first_observation;}

      //ds landmark coordinates update
      void update(const PointCoordinates& coordinates_in_world_,
                  const real& depth_meters_ = 1);
      void update(const PointCoordinates& coordinates_in_world_,
                  const cv::Mat& descriptor_left_,
                  const cv::Mat& descriptor_right_,
                  const real& depth_meters_);

      //ds reset landmark coordinates to a certain position
      void resetCoordinates(const PointCoordinates& coordinates_);

      void createNewItem(const PointCoordinates& spatials_in_world_);
      void releaseItem();
      LandmarkItem* currentItem() {return _current_item;}
      const Count numberOfUpdates() const {return _number_of_updates;}
      const Count numberOfFailedUpdates() const {return _number_of_failed_updates;}

    protected:

      //ds point with index
      const Identifier _index;
      PointCoordinates _coordinates = PointCoordinates::Zero();

      AppearancePtrVector _descriptor_track;
      FramePoint* _first_observation = 0;
      bool _is_validated = false;
      bool _is_active    = false;
      bool _is_optimized = false;
      bool _is_by_vision = false;
      bool _is_contained = true;
      bool _is_closed    = false;

      //ds landmark coordinates optimization
      std::vector<std::pair<real, PointCoordinates>> _measurements_test;
      PointCoordinates _coordinates_average_previous;
      Count _number_of_updates        = 0;
      Count _number_of_failed_updates = 0;

      //ds visualization
      bool _is_in_loop_closure_query     = false;
      bool _is_in_loop_closure_reference = false;

      LandmarkItem* _current_item = 0;

    //ds grant access to landmark producer
    friend WorldMap;

    private:
      static Identifier _instances;
  };
  
  typedef std::vector<Landmark*> LandmarkPtrVector;
  typedef std::pair<int, Landmark*> LandmarkPtrMapElement;

  class LandmarkPtrMap: public std::map<int, Landmark*> {
  public:
    Landmark* get(int index);
    void put(Landmark* landmark);
    void clearActive(); 
  };
}
