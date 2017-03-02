#include "gt_keyframe.h"

#include "srrg_core_map/image_map_node.h"
#include "srrg_core_map/pinhole_camera_info.h"

namespace gslam {
  using namespace srrg_core;
  using namespace srrg_core_map;
  using namespace srrg_boss;

  KeyFrame::KeyFrame(Frame* frame_for_context_, 
		                 FramePtrVector& frames_): Frame(frame_for_context_) {
    _keyframe = this;
    _items.clear();
    _appearances.clear();
    _matches.clear();
    _subcontext.clear();

    //ds keep track of added landmarks in order to add them only once
    std::set<Landmark*> landmarks_added_to_context;

    //ds create item context for this keyframe: loop over all frames
    for (const Frame* frame: frames_) {
      const TransformMatrix3D& frame_to_context = frame_for_context_->worldToRobot()*frame->robotToWorld();
      for (FramePoint* frame_point: frame->points()) {

        //ds buffer current landmark
        Landmark* landmark = frame_point->landmark();

        //ds context item requirements TODO enable proper alignment for vision landmarks
        if (landmark                                    &&
            landmark->isValidated()                     &&
            !landmark->isByVision()                     &&
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
      std::cerr << "KeyFrame::KeyFrame|WARNING: low item number: " << landmarks_added_to_context.size() << std::endl;
    }

    //ds propagate keyframe transform to contained frames
    for (Frame* frame: frames_) {
      if (frame != frame_for_context_) {
        frame->setKeyframe(this);
        _subcontext.push_back(frame);
      }
    }

    //ds clear input
    frames_.clear();
  }

  KeyFrame::~KeyFrame() {

    //ds free all items and their sub elements (e.g. appearances)
    for (const LandmarkItem* item: _items) {
      delete item;
    }
    _items.clear();
    _appearances.clear();
    _matches.clear();
  }

  void KeyFrame::updateSubContext() {

    //ds update robot poses for all contained frames
    for (Frame* frame: _subcontext) {
      assert(frame != this);
      frame->setRobotToWorld(this->robotToWorld()*frame->frameToKeyframe());
    }
  }

  void KeyFrame::write(srrg_boss::Serializer* serializer_, MapNodeList* nodes_, BinaryNodeRelationSet* node_relations_) const {

    //ds fill nodes and relations
    MapNode* node_new      = 0;
    MapNode* node_previous = 0;
    for (Frame* frame: _subcontext) {

     //ds fetch previous node
     if (nodes_->size() > 0) {
       node_previous = nodes_->rbegin()->get();
     }

     //ds allocate a new node
     node_new = new ImageMapNode(static_cast<const Eigen::Isometry3f>(frame->robotToWorld()), 0, "rgbd", frame->sequenceNumberRaw());
     nodes_->addElement(node_new);

      //ds add relation if available
      if (node_previous) {
        BinaryNodeRelation* node_relation = new BinaryNodeRelation();
        node_relation->setFrom(node_previous);
        node_relation->setTo(node_new);
        node_relation->setTransform(node_previous->transform().inverse()*node_new->transform());
        node_relations_->insert(std::tr1::shared_ptr<BinaryNodeRelation>(node_relation));
      }
    }

    //ds add last (the local map origin)
    MapNode* node_local_map_origin = new ImageMapNode(static_cast<const Eigen::Isometry3f>(this->robotToWorld()), 0, "rgbd", this->sequenceNumberRaw());
    nodes_->addElement(node_local_map_origin);
    BinaryNodeRelation* node_relation = new BinaryNodeRelation();
    node_relation->setFrom(node_previous);
    node_relation->setTo(node_local_map_origin);
    node_relation->setTransform(node_previous->transform().inverse()*node_local_map_origin->transform());
    node_relations_->insert(std::tr1::shared_ptr<BinaryNodeRelation>(node_relation));

    //ds allocate a local map handle
    Eigen::Isometry3f world_to_robot = node_local_map_origin->transform().inverse();
    LocalMap* local_map              = new LocalMap(node_local_map_origin->transform());
    local_map->nodes()               = *nodes_;
    local_map->relations()           = *node_relations_;

    //ds fill nodes (frames)
    for (MapNodeList::iterator it = nodes_->begin(); it !=  nodes_->end(); ++it) {
      MapNode* node = it->get();
      node->parents().insert(local_map);
      node->setTransform(world_to_robot*node->transform());
    }

    //ds fill relations (trajectory edges)
    for (BinaryNodeRelationSet::iterator it = node_relations_->begin(); it != node_relations_->end(); ++it) {
      it->get()->setParent(local_map);
    }

    //ds set points in world frame
    Cloud* landmark_cloud = this->generateCloud();
    local_map->setCloud(landmark_cloud);

    //ds serialize local map
    for (MapNodeList::iterator it = local_map->nodes().begin(); it != local_map->nodes().end(); ++it) {
      serializer_->writeObject(*it->get());
    }
    for (BinaryNodeRelationSet::iterator it = local_map->relations().begin(); it != local_map->relations().end(); ++it) {
      serializer_->writeObject(*it->get());
    }
    serializer_->writeObject(*local_map);

    //ds release temporary objects
    delete landmark_cloud;
  }

  Cloud* KeyFrame::generateCloud() const {

    //ds cloud to be filled
    Cloud* cloud = new Cloud();

    //ds loop over all landmarks
    for (const LandmarkItem* item: _items) {
      assert(item->landmark() != 0);

      //ds if landmark is contained in the map
      if (item->landmark()->isContained()) {
        cloud->push_back(RichPoint(item->landmark()->coordinates().cast<float>()));
      }
    }
    return cloud;
  }

} //namespace gtracker
