#include "local_map.h"

namespace proslam {
  using namespace srrg_core;

  LocalMap::LocalMap(Frame* frame_for_context_, 
		                 FramePtrVector& frames_): Frame(frame_for_context_) {
    _local_map = this;
    _items.clear();
    _appearances.clear();
    _matches.clear();
    _subcontext.clear();

    //ds keep track of added landmarks in order to add them only once
    std::set<Landmark*> landmarks_added_to_context;

    //ds create item context for this local map: loop over all frames
    for (const Frame* frame: frames_) {
      const TransformMatrix3D& frame_to_context = frame_for_context_->worldToRobot()*frame->robotToWorld();
      for (FramePoint* frame_point: frame->points()) {

        //ds buffer current landmark
        Landmark* landmark = frame_point->landmark();

        //ds context item requirements TODO enable proper alignment for vision landmarks
        if (landmark                                   &&
            landmark->isValidated()                    &&
            landmark->isClose()                        &&
            !landmarks_added_to_context.count(landmark)) {

          //ds bucket the item and transfer ownership from landmark to current context
          LandmarkItem* item = landmark->currentItem();
          assert(item != 0);

          const PointCoordinates& coordinates_in_context = frame_to_context*frame_point->robotCoordinates();
          item->addSpatials(coordinates_in_context);
          item->setContext(this);
          _items.push_back(item);
          _appearances.insert(_appearances.begin(), item->appearances().begin(), item->appearances().end());
          landmarks_added_to_context.insert(landmark);

          //ds take ownership from landmark view: forces the landmark to generate a new, decoupled view
          landmark->releaseItem();
        }
      }
    }

    //ds check for low item counts
    if (_minimum_number_of_items > landmarks_added_to_context.size()) {
      std::cerr << "LocalMap::LocalMap|WARNING: low item number: " << landmarks_added_to_context.size() << std::endl;
    }

    //ds propagate keyframe transform to contained frames
    for (Frame* frame: frames_) {
      if (frame != frame_for_context_) {
        frame->setLocalMap(this);
        _subcontext.push_back(frame);
      }
    }

    //ds clear input
    frames_.clear();
  }

  LocalMap::~LocalMap() {

    //ds free all items and their sub elements (e.g. appearances)
    for (const LandmarkItem* item: _items) {
      delete item;
    }
    _items.clear();
    _appearances.clear();
    _matches.clear();
  }

  void LocalMap::updateSubContext() {

    //ds update robot poses for all contained frames
    for (Frame* frame: _subcontext) {
      assert(frame != this);
      frame->setRobotToWorld(this->robotToWorld()*frame->frameToLocalMap());
    }
  }
}
