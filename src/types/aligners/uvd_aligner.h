#pragma once
#include "types/contexts/gt_frame.h"
#include "base_aligner.h"

namespace gslam {

  class UVDAligner: public BaseAligner6_3 {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //ds object handling
    protected:

      //ds instantiation controlled by aligner factory
      UVDAligner(): BaseAligner6_3(1e-5, 4.0) {}
      UVDAligner(const gt_real& error_delta_for_convergence_,
                 const gt_real& maximum_error_kernel_): BaseAligner6_3(error_delta_for_convergence_, maximum_error_kernel_) {}
      ~UVDAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity TODO purify this
      virtual void init(BaseContext* context_, const TransformMatrix3D& initial_guess_robot_to_world_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      virtual void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      virtual void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      virtual void converge();

      //ds additional accessors
    public:

      //ds getters/setters
      const TransformMatrix3D robotToWorld() const {return _robot_pose;}
      const TransformMatrix3D worldToRobot() const {return _inverse_robot_pose;}

    //ds aligner specific
    protected:

      //ds context
      Frame* _context = 0;

      //ds objective
      TransformMatrix3D _inverse_robot_pose      = TransformMatrix3D::Identity();
      TransformMatrix3D _robot_pose              = TransformMatrix3D::Identity();

      //ds criteria
      gt_real _weight_depth                      = 1.0;

    //ds grant access to factory: ctor/dtor
    friend AlignerFactory;

  }; //class UVDAligner
} //namespace gslam
