#pragma once

#include "gt_tracking_context.h"
#include "types/aligners/aligner_factory.h"

namespace gslam {
  class WorldContext {

  //ds object handling
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    WorldContext();
    ~WorldContext();

  //ds access
  public:

    TrackingContext* currentTrackingContext() {return _current_subcontext;}

    void createNewTrackingContext();

    //ds merges the current context with a reference
    void merge(const bool& use_gui_);

    //ds extracts merge candidates from a closure to buffer
    void add(CorrespondenceCollection* match_);

    //ds checks status
    const bool isMergeable();

    //ds save maps to file
    void write() const;

    //ds dump trajectory to file (in KITTI benchmark format only for now)
    void writeTrajectory(const std::string& filename_ = "") const;

    //ds specify savefile path
    void setMapSavefilepath(const std::string& map_savefile_path_);

    //ds aligner configuration
    XYZAligner* aligner() {return _aligner;}

  //ds helpers
  protected:

    const TransformMatrix3D getTransform(const TrackingContext* tracking_context_query_,
                                         const TrackingContext* tracking_context_reference_) const;
    const TransformMatrix3D getTransform(CorrespondenceCollection* collection_,
                                         const TransformMatrix3D& transform_query_to_reference_prior) const;

  protected:

    TrackingContext* _current_subcontext = 0;
    TrackingContextPointerMap _subcontexts;
    CorrespondenceMultiMap _merges_queries;
    const Count _minimum_number_of_matches_for_merge = 200;
    bool _added_closure = false;
    XYZAligner* _aligner = 0;

    //ds map output
    srrg_boss::Serializer* _serializer;

    CREATE_CHRONOMETER(overall)

  }; //class WorldContext
} //namespace gtracker
