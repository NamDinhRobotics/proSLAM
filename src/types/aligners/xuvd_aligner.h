#pragma once
#include "uvd_aligner.h"

//ds TODO this class is to be deleted
namespace gslam {

  class XUVDAligner: public UVDAligner {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //ds object handling
    protected:

      //ds instantiation controlled by aligner factory
      XUVDAligner(): UVDAligner(1e-5, 36.0) {}
      ~XUVDAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity TODO purify this
      void init(BaseContext* context_, const TransformMatrix3D& initial_guess_robot_to_world_ = TransformMatrix3D::Identity());

      //ds solve alignment problem for one round: to be called inside converge
      void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      void converge();

    //ds aligner specific
    protected:

      bool _one_round_to_do = true;

    //ds grant access to factory: ctor/dtor
    friend AlignerFactory;

  }; //class UVDAligner
} //namespace gslam
