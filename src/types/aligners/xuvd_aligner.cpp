#include "xuvd_aligner.h"

#include "srrg_types/types.hpp"
#include "types/items/gt_landmark.h"

namespace gslam {

  //ds initialize aligner with minimal entity (TODO purify this, currently Frame)
  void XUVDAligner::init(BaseContext* context_, const TransformMatrix3D& initial_guess_robot_to_world_) {
    UVDAligner::init(context_);
    _one_round_to_do                   = true;
  }

  //ds solve alignment problem for one round
  void XUVDAligner::oneRound(const bool& ignore_outliers_) {
    if (_one_round_to_do) {

//      //ds buffer old robot pose
//      TransformMatrix3D inverse_robot_pose_original = _inverse_robot_pose;

      //ds compute new pose
//      _H.setZero();
//      _b.setZero();
      linearize(ignore_outliers_);
//      _H += _damping*BaseAligner6_3::AlignmentMatrix::Identity();
//      BaseAligner6_3::AlignmentVector dx = _H.ldlt().solve(-_b);
//      _inverse_robot_pose = srrg_core::v2tEuler(static_cast<const srrg_core::Vector6f&>(dx)).cast<gt_real>()*_inverse_robot_pose;
//      BaseAligner6_3::InformationMatrix R = _inverse_robot_pose.linear();
//      BaseAligner6_3::InformationMatrix E = R.transpose() * R;
//      E.diagonal().array() -= 1;
//      _inverse_robot_pose.linear() -= 0.5 * R * E;
//
//      //ds enforce prior transform
//      _inverse_robot_pose.linear()      = inverse_robot_pose_original.linear();
//      _inverse_robot_pose.translation() = inverse_robot_pose_original.translation();
//      _robot_pose = _inverse_robot_pose.inverse();
//
//      //ds update projections
//      linearize(ignore_outliers_);
    }
    _one_round_to_do = false;
  }

  void XUVDAligner::converge() {
    oneRound(false);
  }
} //namespace gslam
