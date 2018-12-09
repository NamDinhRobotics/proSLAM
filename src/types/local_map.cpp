#include "local_map.h"

namespace proslam {

Count LocalMap::_instances = 0;

LocalMap::LocalMap(FramePointerVector& frames_,
                   const LocalMapParameters* parameters_,
                   LocalMap* local_map_root_,
                   LocalMap* local_map_previous_): _identifier(_instances),
                                                   _root(local_map_root_),
                                                   _previous(local_map_previous_),
                                                   _parameters(parameters_) {
  assert(!frames_.empty());
  ++_instances;

  //ds clear structures
  clear();

  //ds if the current local map is preceded by another local map
  if (local_map_previous_) {

    //ds link the preceeding local map to this one
    _previous->setNext(this);
  }

  //ds define keyframe (currently the last frame)
  _keyframe = frames_.back();
  _keyframe->setIsKeyframe(true);

  //ds define local map position relative in the world based on the key frame
  const TransformMatrix3D& world_to_local_map = _keyframe->worldToRobot();

  //ds keep track of added landmarks in order to add them only once
  std::set<Identifier> landmarks_added;

  //ds preallocate bookkeeping with maximum allowed landmarks
  const Count maximum_number_of_landmarks = _parameters->maximum_number_of_landmarks;

  //ds create item context for this local map: loop over all frames
  for (Frame* frame: frames_) {
    const TransformMatrix3D& robot_to_local_map = world_to_local_map*frame->robotToWorld();
    frame->setLocalMap(this);
    frame->setRobotToLocalMap(robot_to_local_map);

    //ds for all framepoints in this frame
    for (FramePoint* frame_point: frame->points()) {

      //ds check for landmark
      Landmark* landmark = frame_point->landmark();

      //ds if we have a landmark and it has not been added yet
      if (landmark && landmarks_added.count(landmark->identifier()) == 0) {

        //ds create HBST matchables based on available landmark descriptors TODO move this operation into a method of the landmark
        HBSTTree::MatchableVector matchables(landmark->_descriptors.size());
        for (Count u = 0; u < matchables.size(); ++u) {
          HBSTMatchable* matchable = new HBSTMatchable(landmark, landmark->_descriptors[u], _identifier);
          matchables[u]            = matchable;
          landmark->_appearance_map.insert(std::make_pair(matchable, matchable));
        }
        landmark->_descriptors.clear();
        landmark->_local_maps.insert(this);

        //ds create a landmark snapshot and add it to the local map
        const PointCoordinates coordinates_in_local_map = world_to_local_map*landmark->coordinates();
        _landmarks.insert(std::make_pair(landmark->identifier(), Closure::LandmarkState(landmark, coordinates_in_local_map)));

        //ds we're only interested in the appearances generated in this local map
        _appearances.insert(_appearances.end(), matchables.begin(), matchables.end());

        //ds block further additions of this landmark
        landmarks_added.insert(landmark->identifier());
      }
    }
  }

  //ds set proper identity transform for keyframe (above loop might cause numerical imprecision)
  _keyframe->setRobotToLocalMap(TransformMatrix3D::Identity());

  //ds check if we have to sparsify the landmarks TODO implement, check how to trim appearance vector as well
  if (_landmarks.size() > maximum_number_of_landmarks) {
    LOG_INFO(std::cerr << "LocalMap::LocalMap|" << _identifier
                       << "|pruning landmarks from: " << landmarks_added.size() << " to: " << maximum_number_of_landmarks << std::endl)

    //ds TODO free unused landmark states
  }

  //ds check for low item counts
  if (_parameters->minimum_number_of_landmarks > landmarks_added.size()) {
    LOG_WARNING(std::cerr << "LocalMap::LocalMap|creating local map with low landmark number: " << landmarks_added.size() << std::endl)
  }

  //ds gobble up frames
  _frames.insert(_frames.end(), frames_.begin(), frames_.end());
}

LocalMap::~LocalMap() {
  clear();
}

void LocalMap::clear() {
  _landmarks.clear();
  _closures.clear();
  _frames.clear();
  _appearances.clear();
}

void LocalMap::replace(Landmark* landmark_old_, Landmark* landmark_new_) {

  //ds remove the old landmark from the local map and check for failure
  if (_landmarks.erase(landmark_old_->identifier()) != 1) {
    LOG_WARNING(std::cerr << "LocalMap::replace|" << _identifier << "|unable to erase old landmark with ID: " << landmark_old_->identifier() << std::endl)
  }

  //ds look if the new landmark is already present (can happen through merging)
  Closure::LandmarkStateMap::iterator iterator = _landmarks.find(landmark_new_->identifier());
  if (iterator != _landmarks.end()) {

    //ds update the entry
    iterator->second.coordinates_in_local_map = _keyframe->worldToRobot()*landmark_new_->coordinates();
  } else {

    //ds create a new entry with updated landmark coordinates
    _landmarks.insert(std::make_pair(landmark_new_->identifier(), Closure::LandmarkState(landmark_new_, _keyframe->worldToRobot()*landmark_new_->coordinates())));
  }
}

void LocalMap::setRobotToWorld(const TransformMatrix3D& robot_to_world_, const bool update_landmark_world_coordinates_) {

  //ds update frame poses for all contained frames
  for (Frame* frame: _frames) {
    frame->setRobotToWorld(robot_to_world_*frame->robotToLocalMap(), false);
  }

  //ds update landmark world coordinates according to this local map estimate
  if (update_landmark_world_coordinates_) {
    for (Closure::LandmarkStateMapElement& element: _landmarks) {
      element.second.landmark->setCoordinates(robot_to_world_*element.second.coordinates_in_local_map);
    }
  }
}
}
