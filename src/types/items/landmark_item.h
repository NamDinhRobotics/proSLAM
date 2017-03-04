#pragma once
#include "appearance.h"
#include "landmark.h"

namespace gslam {

  class LandmarkItem {

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //ds allocate item with spatials only
    LandmarkItem(Landmark* landmark_,
                 const PointCoordinates& spatials_): _landmark(landmark_) {
      addSpatials(spatials_);
      _appearances.clear();
      assert(_landmark != 0);
    }

    //ds allocate item with spatials and appearance
    LandmarkItem(Landmark* landmark_,
                 const PointCoordinates& spatials_,
                 const cv::Mat& descriptor_): LandmarkItem(landmark_, spatials_) {
      addAppearance(descriptor_);
    }

    ~LandmarkItem() {
      //ds appearance cleanup is done by landmarks, as they own them (necessary if accumulation is desired) TODO purify
      //for (const Appearance* appearance: _appearances) {
      //  delete appearance;
      //}
      _appearances.clear();
    }

    LandmarkItem() = delete;

    inline Landmark* landmark() {return _landmark;} //ds TODO restrict readwrite access
    inline const Landmark* landmark() const {return _landmark;}
    inline const PointCoordinates& spatials() const {return _spatials;}
    inline const AppearancePtrVector& appearances() const {return _appearances;}
    inline void addAppearance(const cv::Mat& descriptor_) {_appearances.push_back(new Appearance(this, descriptor_));}
    inline void addSpatials(const PointCoordinates& spatials_) {_spatials = spatials_;} //ds currently always overwriting TODO implement collection behavior
    inline void setContext(const KeyFrame* context_) {_context = context_;}

  protected:

    Landmark* _landmark       = 0;    //ds must be set at construction
    const KeyFrame* _context  = 0;    //ds every item has NO CONTEXT upon creation and EXACTLY ONE CONTEXT once claimed
    PointCoordinates _spatials;
    AppearancePtrVector _appearances;

  };

  typedef std::vector<LandmarkItem*> LandmarkItemPointerVector;
}
