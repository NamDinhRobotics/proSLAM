#include "local_map.h"

namespace proslam {
  using namespace srrg_core;

  LocalMap::LocalMap(Frame* frame_for_context_, 
		                 FramePointerVector& frames_): Frame(frame_for_context_) {

    //ds link anchor frame
    Frame::_is_local_map_anchor = true;
    Frame::setLocalMap(this);

    //ds clear structures
    _landmarks.clear();
    _appearances.clear();
    _closures.clear();
    _frames.clear();

    //ds keep track of added landmarks in order to add them only once
    std::set<const Landmark*> landmarks_added_to_context;

    //ds create item context for this local map: loop over all frames
    for (Frame* frame: frames_) {
      const TransformMatrix3D& frame_to_context = frame_for_context_->worldToRobot()*frame->robotToWorld();
      for (FramePoint* frame_point: frame->points()) {

        //ds buffer current landmark
        Landmark* landmark = frame_point->landmark();

        //ds context item requirements
        if (landmark                                  &&
            landmark->areCoordinatesValidated()       &&
            landmark->isNear()                        &&
            !landmarks_added_to_context.count(landmark)) {

          //ds bucket the item and transfer ownership from landmark to current context
          Landmark::State* landmark_state = landmark->state();
          assert(landmark_state != 0);

          landmark_state->robot_coordinates = frame_to_context*frame_point->robotCoordinates();
          landmark_state->local_map = this;
          _landmarks.push_back(landmark_state);
          _appearances.insert(_appearances.begin(), landmark_state->appearances.begin(), landmark_state->appearances.end());
          landmarks_added_to_context.insert(landmark);

          //ds take ownership from landmark view: forces the landmark to generate a new, decoupled view
          landmark->refreshState();
          landmark->setLocalMap(this);
        }
      }
    }

    //ds check for low item counts
    if (_minimum_number_of_landmarks > landmarks_added_to_context.size()) {
      std::cerr << "LocalMap::LocalMap|WARNING: creating local map with low landmark number: " << landmarks_added_to_context.size() << std::endl;
    }

    //ds remove the last frame (being the one become this local map)
    frames_.pop_back();

    //ds propagate keyframe transform to contained frames
    for (Frame* frame: frames_) {
      frame->setLocalMap(this);
      _frames.push_back(frame);
    }
  }

  //ds cleanup of dynamic structures
  LocalMap::~LocalMap() {

    //ds free all items and their sub elements (e.g. appearances)
    for (const Landmark::State* item: _landmarks) {
      delete item;
    }
    _landmarks.clear();
    _appearances.clear();
    _closures.clear();
  }

  void LocalMap::setRobotToWorld(const TransformMatrix3D& robot_to_world_) {

    //ds set own position
    Frame::setRobotToWorld(robot_to_world_);

    //ds update robot poses for all contained frames
    for (Frame* frame: _frames) {
      frame->setRobotToWorld(robot_to_world_*frame->frameToLocalMap());
    }
  }
}
