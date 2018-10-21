#include "local_map.h"

namespace proslam {

Count LocalMap::_instances = 0;

LocalMap::LocalMap(FramePointerVector& frames_,
                   const LocalMapParameters* parameters_,
                   LocalMap* local_map_root_,
                   LocalMap* local_map_previous_): _identifier(_instances),
                                                   _local_map_to_world(TransformMatrix3D::Identity()),
                                                   _world_to_local_map(TransformMatrix3D::Identity()),
                                                   _root(local_map_root_),
                                                   _previous(local_map_previous_),
                                                   _next(0),
                                                   _parameters(parameters_) {
  assert(frames_.size() > 0);
  ++_instances;

  //ds clear structures
  clear();

  //ds if the current local map is preceded by another local map
  if (local_map_previous_) {

    //ds link the preceeding local map to this one
    _previous->setNext(this);
  }

  //ds define keyframe
  _keyframe = frames_.back();

  //ds define local map position relative in the world (currently using last frame's pose)
  const TransformMatrix3D& world_to_local_map = _keyframe->worldToRobot();

  //ds set local map pose
  setWorldToLocalMap(world_to_local_map);

  //ds keep track of added landmarks in order to add them only once
  LandmarkPointerSet landmarks_added_to_context;

  //ds create item context for this local map: loop over all frames
  for (Frame* frame: frames_) {
    const TransformMatrix3D& frame_to_local_map = world_to_local_map*frame->robotToWorld();
    frame->setLocalMap(this);
    frame->setFrameToLocalMap(frame_to_local_map);
    for (FramePoint* frame_point: frame->points()) {

      //ds buffer current landmark
      Landmark* landmark = frame_point->landmark();

      //ds context item requirements
      if (landmark                                  &&
          !landmarks_added_to_context.count(landmark)) {

        //ds bucket the item and transfer ownership from landmark to current context
        Landmark::State* landmark_state = landmark->state();
        assert(landmark_state != 0);

        //ds update state position in local map
        landmark_state->coordinates_in_local_map = world_to_local_map*landmark_state->world_coordinates;
        landmark_state->local_map = this;
        _landmarks.push_back(landmark_state);
        _appearances.insert(_appearances.begin(), landmark_state->appearances.begin(), landmark_state->appearances.end());
        landmarks_added_to_context.insert(landmark);

        //ds take ownership from landmark view: forces the landmark to generate a new, decoupled view
        landmark->renewState();
      }
    }
  }

  //ds check for low item counts
  if (_parameters->minimum_number_of_landmarks > landmarks_added_to_context.size()) {
    LOG_WARNING(std::cerr << "LocalMap::LocalMap|creating local map with low landmark number: " << landmarks_added_to_context.size() << std::endl)
  }

  //ds add frames to the local map
  _frames.insert(_frames.end(), frames_.begin(), frames_.end());

  //ds promote local map center to keyframe (currently the last frame)
  _keyframe->setIsKeyframe(true);
}

LocalMap::~LocalMap() {
  clear();
}

void LocalMap::clear() {
  for (const Landmark::State* landmark_state: _landmarks) {
    delete landmark_state;
  }
  for (const Closure& closure: _closures) {
    for (const LandmarkCorrespondence* landmark_correspondence: closure.landmark_correspondences) {
      delete landmark_correspondence;
    }
  }
  _landmarks.clear();
  _appearances.clear();
  _closures.clear();
  _frames.clear();
}

void LocalMap::update(const TransformMatrix3D& local_map_to_world_) {
  setLocalMapToWorld(local_map_to_world_);

  //ds update frame poses for all contained frames
  for (Frame* frame: _frames) {
    frame->setRobotToWorld(_local_map_to_world*frame->frameToLocalMap());
  }
}
}
