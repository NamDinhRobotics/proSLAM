#pragma once
#include "types/frame.h"
#include "base_aligner.h"

namespace proslam {

  //ds implements an interface for frame-to-frame aligner to be used inside the tracker
  class BaseFrameAligner: public BaseAligner {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseFrameAligner() {}

    BaseFrameAligner(const real& error_delta_for_convergence_):BaseAligner(error_delta_for_convergence_) {}

    BaseFrameAligner(const real& error_delta_for_convergence_,
                     const real& maximum_error_kernel_): BaseAligner(error_delta_for_convergence_,
                                                                     maximum_error_kernel_) {}

    BaseFrameAligner(const real& error_delta_for_convergence_,
                     const real& maximum_error_kernel_,
                     const real& damping_): BaseAligner(error_delta_for_convergence_,
                                                        maximum_error_kernel_,
                                                        damping_) {}

    BaseFrameAligner(const real& error_delta_for_convergence_,
                     const real& maximum_error_kernel_,
                     const real& damping_,
                     const uint64_t& maximum_number_of_iterations_): BaseAligner(error_delta_for_convergence_,
                                                                                 maximum_error_kernel_,
                                                                                 damping_,
                                                                                 maximum_number_of_iterations_) {}

    virtual ~BaseFrameAligner() {};

  //ds functionality
  public:

    //ds pure virtual
    virtual void initialize(Frame* frame_, const TransformMatrix3D& robot_to_world_) = 0;

  //ds setters/getters
  public:

    void setWeightFramepoint(const real& weight_framepoint_) {_weight_framepoint = weight_framepoint_;}
    void setMaximumDepthNearMeters(const real& maximum_depth_near_meters_) {_maximum_depth_near_meters = maximum_depth_near_meters_;}
    void setMaximumDepthFarMeters(const real& maximum_depth_far_meters_) {_maximum_depth_far_meters = maximum_depth_far_meters_;}
    inline const TransformMatrix3D& robotToWorld() const {return _robot_to_world;}
    inline const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}

  //ds attributes
  protected:

    //ds alignment context
    Frame* _frame = 0;

    //ds objective
    TransformMatrix3D _world_to_camera = TransformMatrix3D::Identity();
    TransformMatrix3D _camera_to_world = TransformMatrix3D::Identity();

    real _weight_framepoint         = 1;
    real _maximum_depth_near_meters = 0;
    real _maximum_depth_far_meters  = 0;

    Count _number_of_rows_image = 0;
    Count _number_of_cols_image = 0;

    //ds context wrapping objects
    TransformMatrix3D _world_to_robot = TransformMatrix3D::Identity();
    TransformMatrix3D _robot_to_world = TransformMatrix3D::Identity();
  };
}
