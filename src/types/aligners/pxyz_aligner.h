#pragma once
#include "../contexts/gt_frame.h"
#include "base_aligner.h"

namespace gslam {

  class PXYZAligner: public BaseAligner6_3 {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //ds object handling
    protected:

      //ds instantiation controlled by aligner factory
      PXYZAligner(): BaseAligner6_3(1e-5, 0.5) {}
      ~PXYZAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity TODO purify this
      void init(BaseContext* context_, const TransformMatrix3D& robot_to_world_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      void converge();

    //ds additional accessors
    public:

      //ds getters/setters
      const TransformMatrix3D robotToWorld() const {return _robot_to_world;}
      const TransformMatrix3D worldToRobot() const {return _world_to_robot;}

    //ds aligner specific
    protected:

      //ds context
      Frame* _context = 0;

      //ds objective
      TransformMatrix3D _robot_to_world  = TransformMatrix3D::Identity();
      TransformMatrix3D _world_to_robot  = TransformMatrix3D::Identity();

    //ds grant access to factory: ctor/dtor
    friend AlignerFactory;

  }; //class PXYZAligner
} //namespace gslam
