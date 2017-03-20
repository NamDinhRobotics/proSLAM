#pragma once
#include "frame.h"

namespace proslam {

  class Landmark {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds converter TODO remove!
    inline static HBSTMatchable::BinaryDescriptor getDescriptor(const cv::Mat& descriptor_cv_) {
      HBSTMatchable::BinaryDescriptor binary_descriptor(DESCRIPTOR_SIZE_BITS);
      for (uint32_t byte_index = 0 ; byte_index < DESCRIPTOR_SIZE_BYTES; ++byte_index) {

        //ds get minimal datafrom cv::mat
        const uchar value = descriptor_cv_.at<uchar>(byte_index);

        //ds get bitstring
        for (uint8_t v = 0; v < 8; ++v) {
          binary_descriptor[byte_index*8+v] = (value >> v) & 1;
        }
      }
      return binary_descriptor;
    }

    //ds landmark snapshot for a local map
    struct Item;
    struct Appearance {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      Appearance(const Item* item_,
                 const cv::Mat& descriptor_cv_): item(item_),
                                                 descriptor_cv(descriptor_cv_),
                                                 descriptor(getDescriptor(descriptor_cv_)) {}

      const Item* item;
      const cv::Mat descriptor_cv;
      const HBSTMatchable::BinaryDescriptor descriptor;
    };
    typedef std::vector<const Appearance*> AppearancePtrVector;

    struct Item {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Item(Landmark* landmark_): landmark(landmark_) {
        appearances.clear();
      }
      ~Item() {
        for (const Appearance* appearance: appearances) {
          delete appearance;
        }
        appearances.clear();
      }

      Landmark* landmark = 0;
      AppearancePtrVector appearances;
      PointCoordinates robot_coordinates;
      LocalMap* local_map = 0;
    };
    typedef std::vector<Item*> ItemPointerVector;

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
    inline Item* item() {return _item;}
    inline void releaseItem() {_item = new Item(this);}

    inline const bool isValidated() const {return _is_validated;}
    inline const bool isActive() const {return _is_active; }
    inline void setIsActive(const bool& is_active) {_is_active = is_active;}
    inline const bool isOptimized() const {return _is_optimized; }
    inline void setIsOptimized(const bool& is_optimized_) {_is_optimized = is_optimized_;}
    inline const bool isClose() const {return _is_close;}
    inline void setIsClose(const bool& is_close_) {_is_close = is_close_;}
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

    const Count numberOfUpdates() const {return _number_of_updates;}
    const Count numberOfFailedUpdates() const {return _number_of_failed_updates;}

  protected:

    //ds point with index
    const Identifier _index;
    PointCoordinates _coordinates;
    Item* _item;

    FramePoint* _first_observation = 0;
    bool _is_validated = false;
    bool _is_active    = false;
    bool _is_optimized = false;
    bool _is_close     = false;
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
