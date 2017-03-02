#include "gt_world_context.h"

#include "ui/gt_viewer_icp.h"
#include "srrg_types/types.hpp"

namespace gslam {
  using namespace srrg_core;
  using namespace srrg_boss;

  WorldContext::WorldContext(): _serializer(new Serializer()) {
    _subcontexts.clear();
    _merges_queries.clear();
    _aligner = static_cast<XYZAligner*>(AlignerFactory::create(AlignerType6_3::xyz));
    assert(_aligner != 0);
    _aligner->setContextDepth(1);
    std::cerr << "WorldContext::WorldContext|constructed" << std::endl;
  }
  WorldContext::~WorldContext() {
    std::cerr <<"WorldContext::WorldContext|destroying" << std::endl;

    //ds free remaininig subcontexts
    for (const TrackingContextPointerMapElement subcontext_element: _subcontexts) {
      delete subcontext_element.second;
    }
    std::cerr <<"WorldContext::WorldContext|destroyed" << std::endl;
  }

  void WorldContext::createNewTrackingContext() {
    CHRONOMETER_START(overall)

    //ds if we have an existing "previous" context set as last position
    if (_current_subcontext != 0) {
      _current_subcontext = new TrackingContext(_current_subcontext);

    //ds otherwise set origin
    } else {
      _current_subcontext = new TrackingContext();
    }

    //ds keep track of all generated contexts
    _subcontexts.insert(std::make_pair(_current_subcontext->index(), _current_subcontext));
    CHRONOMETER_STOP(overall)
  }

  //ds merges the current context with a reference
  void WorldContext::merge(const bool& use_gui_) {
    CHRONOMETER_START(overall)

    //ds loop over all multimap merges
    std::vector<Identifier> multimap_merges_completed(0);
    std::vector<Identifier> multimap_merges_failed(0);
    for(CorrespondenceMultiMapElement merges: _merges_queries) {

      //ds buffer for readability
      CorrespondenceMap& merges_for_query = merges.second;

      //ds start optional merging process - insufficient merges are kept
      bool merged       = false;
      bool merge_failed = false;
      for (CorrespondenceMapElement merge_query: merges_for_query) {
        if (merge_query.second->correspondences.size() > _minimum_number_of_matches_for_merge) {
          const Identifier index_context_to_absorb = _current_subcontext->index();
          assert(index_context_to_absorb != merge_query.first);
          std::cerr << "merging contexts: " << index_context_to_absorb << " > " << merge_query.first << " carrying key frames: " << _current_subcontext->keyframes().size() << std::endl;

          //ds fetch reference context (in here we have write access)
          TrackingContext* context_reference = _subcontexts.at(merge_query.first);

          //ds final transform
          TransformMatrix3D transform_query_world_to_reference_world = TransformMatrix3D::Identity();

          //ds transform priors
          const TransformMatrix3D& transform_query_world_to_reference_world_prior_context = getTransform(_current_subcontext, context_reference);
          const TransformMatrix3D& transform_query_world_to_reference_world_prior_closure = (merge_query.second->keyframe_reference->robotToWorld()*merge_query.second->transform_frame_query_to_frame_reference)*merge_query.second->keyframe_query->worldToRobot();

          //ds attempt transform retrieval with 2 priors:
          try {

            //ds retrieve rigid transform for this merge
            transform_query_world_to_reference_world = getTransform(merge_query.second, transform_query_world_to_reference_world_prior_context);
          }
          catch(const ExceptionNoTransformFound& exception_) {

            std::cerr << "merging with context prior failed for contexts: " << index_context_to_absorb << " > " << merge_query.first << " (" << merge_query.second->correspondences.size() << ") " << exception_.what() << std::endl;
            try {

              //ds retrieve rigid transform for this merge
              transform_query_world_to_reference_world = getTransform(merge_query.second, transform_query_world_to_reference_world_prior_closure);

            }
            catch(const ExceptionNoTransformFound& exception_) {
              std::cerr << "skipping merging contexts: " << index_context_to_absorb << " > " << merge_query.first << " (" << merge_query.second->correspondences.size() << ") " << exception_.what() << std::endl;
              merge_failed = true;
              continue;
            }
          }

          //ds merge query into reference context: ownership of all objects (landmarks, keyframes..) is transferred to reference context
          context_reference->absorb(_current_subcontext, transform_query_world_to_reference_world);

          //ds update current to absorbing context
          _current_subcontext = context_reference;
          _subcontexts.erase(index_context_to_absorb);
          assert(_current_subcontext->index() == context_reference->index());

          //ds draw merge
          if (use_gui_) {
            for (const Correspondence* match: merge_query.second->correspondences) {
              _current_subcontext->landmarks().get(match->item_query->landmark()->index())->setIsInMapMergeQuery(true);
              _current_subcontext->landmarks().get(match->item_reference->landmark()->index())->setIsInMapMergeReference(true);
            }
          }

          //ds currently only one multimap merge per call (TODO implement correct reference update)
          std::cerr << "merged contexts: " << index_context_to_absorb << " > " << _current_subcontext->index() << std::endl;
          merged = true;
          break;
        } else {
          std::cerr << "skipping merging contexts: " << _current_subcontext->index() << " > " << merge_query.first << " (" << merge_query.second->correspondences.size() << ") insufficient matches" << std::endl;
        }
      }

      //ds if merged: clear merged queries
      if (merged) {
        multimap_merges_completed.push_back(merges.first);
        break;
      }

      //ds register failed merges
      if (merge_failed) {
        multimap_merges_failed.push_back(merges.first);
      }
    }

    //ds clear successful queries from multimap
    for (const Identifier& index_query: multimap_merges_completed) {
      for (CorrespondenceMapElement merge_query: _merges_queries.at(index_query)) {
        delete merge_query.second;
      }
      _merges_queries.at(index_query).clear();
      _merges_queries.erase(index_query);
    }

    //ds also clear unsuccesful queries to remove potential invalid correspondence collections
    for (const Identifier& index_query: multimap_merges_failed) {
      for (CorrespondenceMapElement merge_query: _merges_queries.at(index_query)) {
        delete merge_query.second;
      }
      _merges_queries.at(index_query).clear();
      _merges_queries.erase(index_query);
    }
    CHRONOMETER_STOP(overall)
  }

  void WorldContext::add(CorrespondenceCollection* collection_query_current_) {
    CHRONOMETER_START(overall)
    const Identifier index_query_context     = collection_query_current_->keyframe_query->trackingContext()->index();
    const Identifier index_reference_context = collection_query_current_->keyframe_reference->trackingContext()->index();

//    //ds deep clone correspondences (the relocalizer will erase the owned correspondences)
//    CorrespondencePointerVector matches_cloned;
//    for (const Correspondence* match: match_->correspondences) {
//
//      //ds only use correspondence if both! landmarks are valid
//      if (match->item_query->landmark()->isValidated() && match->item_reference->landmark()->isValidated()) {
//        matches_cloned.push_back(new Correspondence(match));
//      }
//    }
//
//    //ds local query world to reference world estimate
//    const TransformMatrix3D transform_query_world_to_reference_world = (match_->keyframe_reference->frameToWorld()*match_->transform_frame_query_to_frame_reference)*match_->keyframe_query->worldToFrame();

    //ds get a deep copy of the context
    assert(collection_query_current_ != 0);

    //ds check if a merge against this reference context is already existing
    try {

      //ds access query map
      CorrespondenceMap& merges = _merges_queries.at(index_query_context);

      try {

        //ds attempt to retrieve an existing collection
        CorrespondenceCollection* collection_query_previous = merges.at(index_reference_context);
        assert(collection_query_previous != 0);

        //ds add correspondence copies
        collection_query_previous->absorb(collection_query_current_);
      }
      catch(const std::out_of_range& /*exception*/) {

        //ds initialize the first merge query for this reference tracking context - ds get a deep copy of the context
        merges.insert(std::make_pair(index_reference_context, new CorrespondenceCollection(collection_query_current_)));
      }
    } catch(const std::out_of_range& /*exception*/) {

      //ds get a deep copy of the context
      CorrespondenceMap merges;
      merges.insert(std::make_pair(index_reference_context, new CorrespondenceCollection(collection_query_current_)));

      //ds initialize the first merge query for this reference tracking context
      _merges_queries.insert(std::make_pair(index_query_context, merges));
    }

    //ds enable merging
    _added_closure = true;
    CHRONOMETER_STOP(overall)
  }

  const bool WorldContext::isMergeable() {
    CHRONOMETER_START(overall)
    if (_added_closure) {
      _added_closure = false;
      for(CorrespondenceMultiMapElement merges: _merges_queries) {
        for (CorrespondenceMapElement merge_query: merges.second) {
          if (merge_query.second->correspondences.size() > _minimum_number_of_matches_for_merge) {
            CHRONOMETER_STOP(overall)
            return true;
          }
        }
      }
    }
    CHRONOMETER_STOP(overall)
    return false;
  }

  //ds save maps to file
  void WorldContext::write() const {
    std::cerr <<"WorldContext::WorldContext|saving map to file" << std::endl;
    for (const TrackingContextPointerMapElement subcontext_element: _subcontexts) {
      subcontext_element.second->write(_serializer);
    }
    std::cerr << "WorldContext::WorldContext|map successfully saved" << std::endl;
  }

  //ds dump trajectory to file (in KITTI benchmark format only for now)
  void WorldContext::writeTrajectory(const std::string& filename_) const {

    //ds expecting complete tracking context
    if (_subcontexts.size() == 1) {

      //ds buffer frame map
      const FramePtrMap& frames(_subcontexts.at(0)->frames());

      //ds construct filename
      std::string filename(filename_);

      //ds if not set
      if (filename_ == "") {

        //ds generate generic filename with timestamp
        filename = "trajectory-"+std::to_string(static_cast<uint64_t>(std::round(srrg_core::getTime())))+".txt";
      }

      //ds open file stream (overwriting)
      std::ofstream outfile_trajectory(filename, std::ifstream::out);
      assert(outfile_trajectory.good());

      //ds for each frame (assuming continuous, sequential indexing)
      for (Index index_frame = 0; index_frame < frames.size(); ++index_frame) {

        //ds buffer transform
        const TransformMatrix3D& robot_to_world = frames.at(index_frame)->robotToWorld();

        //ds dump transform according to KITTI format
        for (uint8_t u = 0; u < 3; ++u) {
          for (uint8_t v = 0; v < 4; ++v) {
            outfile_trajectory << robot_to_world(u,v) << " ";
          }
        }
        outfile_trajectory << "\n";
      }
      outfile_trajectory.close();
      std::cerr << "WorldContext::WorldContext|saved trajectory to: " << filename << std::endl;
    } else {
      std::cerr << "WorldContext::WorldContext|unable to save trajectory, found non-merged contexts" << std::endl;
    }
  }

  //ds specify savefile path
  void WorldContext::setMapSavefilepath(const std::string& map_savefile_path_) {
    assert(_serializer != 0);
    assert(map_savefile_path_.length() > 0);
    _serializer->setFilePath(map_savefile_path_);
    _serializer->setBinaryPath(map_savefile_path_ + ".d/<classname>.<nameAttribute>.<id>.<ext>");
  }

  const TransformMatrix3D WorldContext::getTransform(const TrackingContext* tracking_context_query_,
                                                     const TrackingContext* tracking_context_reference_) const {
    TransformMatrix3D transform_query_to_reference = TransformMatrix3D::Identity();
    const TrackingContext* context = tracking_context_reference_->next();

    //ds iterate over all contexts until we reach the query
    while (context != tracking_context_query_) {
      transform_query_to_reference = transform_query_to_reference*context->currentToPreviousContext();
      context = context->next();
    }

    //ds add query to finalize the transform
    return transform_query_to_reference*context->currentToPreviousContext();
  }

  const TransformMatrix3D WorldContext::getTransform(CorrespondenceCollection* collection_,
                                                     const TransformMatrix3D& transform_query_to_reference_prior_) const {

    //ds solve aligment transform
    _aligner->init(collection_, transform_query_to_reference_prior_);
    _aligner->converge();

    //ds if converged
    if (collection_->is_valid) {
      return _aligner->currentToReference();
    } else {
      throw ExceptionNoTransformFound("unable to find transform");
    }
  }
} //namespace gtracker
