#include "local_map.h"

namespace proslam {

Count LocalMap::_instances = 0;

LocalMap::LocalMap(FramePointerVector& frames_,
                   const LocalMapParameters* parameters_,
                   LocalMap* local_map_root_,
                   LocalMap* local_map_previous_): _identifier(_instances),
                                                   _root(local_map_root_),
                                                   _previous(local_map_previous_),
                                                   _next(nullptr),
                                                   _parameters(parameters_) {
  assert(!frames_.empty());
  ++_instances;

  //ds clear structures
  _landmark_states.clear();
  _closures.clear();
  _frames.clear();

  //ds if the current local map is preceded by another local map
  if (local_map_previous_) {

    //ds link the preceeding local map to this one
    _previous->setNext(this);
  }

  //ds define keyframe (currently the last frame)
  _frames   = frames_;
  _keyframe = _frames.back();
  _keyframe->setIsKeyframe(true);

  //ds define local map position relative in the world (currently using last frame's pose)
  setLocalMapToWorld(_keyframe->robotToWorld(), false);

  //ds keep track of added landmarks in order to add them only once
  std::set<Identifier> landmark_states_added;

  //ds preallocate bookkeeping with maximum allowed landmarks
  const Count maximum_number_of_landmarks = _parameters->maximum_number_of_landmarks;
  _landmark_states.resize(maximum_number_of_landmarks);

  //ds create item context for this local map: loop over all frames
  for (Frame* frame: frames_) {
    const TransformMatrix3D& frame_to_local_map = _world_to_local_map*frame->robotToWorld();
    frame->setLocalMap(this);
    frame->setFrameToLocalMap(frame_to_local_map);

    //ds for all framepoints in this frame
    for (FramePoint* frame_point: frame->points()) {

      //ds check for landmark
      Landmark* landmark = frame_point->landmark();

      //ds if we have a landmark and it has not been added yet
      if (landmark && landmark_states_added.count(landmark->identifier()) == 0) {
        const Index number_of_added_landmarks = landmark_states_added.size();

        //ds temporarily extend landmark buffer (which is sparsified later to fit the original maximum size)
        if (number_of_added_landmarks == maximum_number_of_landmarks) {
          _landmark_states.resize(_landmark_states.size()+maximum_number_of_landmarks);
        }

        //ds create HBST matchables
        HBSTTree::MatchableVector matchables(landmark->appearances().size());
        for (Count u = 0; u < matchables.size(); ++u) {
          matchables[u] = new HBSTMatchable(landmark, landmark->appearances()[u], _identifier);
        }

        //ds create a landmark snapshot and add it to the local map
        _landmark_states[number_of_added_landmarks] = new Landmark::State(landmark, matchables, _world_to_local_map*landmark->coordinates());

        //ds block further additions of this landmark
        landmark_states_added.insert(landmark->identifier());
      }
    }
  }
  _landmark_states.resize(landmark_states_added.size());

  //ds check if we have to sparsify the landmarks TODO implement
  if (_landmark_states.size() > maximum_number_of_landmarks) {
    LOG_INFO(std::cerr << "LocalMap::LocalMap|" << _identifier
                       << "|pruning landmarks from: " << landmark_states_added.size() << " to: " << maximum_number_of_landmarks << std::endl)

    //ds TODO free unused landmark states
  }

  //ds add appearances for all remaining landmarks
  for (Landmark::State* landmark_state: _landmark_states) {
    _appearances.insert(_appearances.end(), landmark_state->appearances.begin(), landmark_state->appearances.end());
    landmark_state->landmark->addState(landmark_state);
  }

  //ds check for low item counts
  if (_parameters->minimum_number_of_landmarks > landmark_states_added.size()) {
    LOG_WARNING(std::cerr << "LocalMap::LocalMap|creating local map with low landmark number: " << landmark_states_added.size() << std::endl)
  }
}

LocalMap::~LocalMap() {
  clear();
}

void LocalMap::clear() {
  for (const Landmark::State* state: _landmark_states) {
    delete state;
  }
  _landmark_states.clear();
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

void LocalMap::setLocalMapToWorld(const TransformMatrix3D& local_map_to_world_, const bool update_landmark_world_coordinates_) {
  _local_map_to_world = local_map_to_world_;
  _world_to_local_map = _local_map_to_world.inverse();

  //ds update landmark world coordinates according to this local map estimate
  if (update_landmark_world_coordinates_) {
    for (Index index = 0; index < _landmark_states.size(); ++index) {
      _landmark_states[index]->landmark->setCoordinates(_local_map_to_world*_landmark_states[index]->coordinates);
    }
  }
}
}
