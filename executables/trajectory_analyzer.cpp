#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

#include "srrg_types/types.hpp"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"

using namespace srrg_core;

//ds computes absolute translation RMSE
const double getAbsoluteTranslationRootMeanSquaredError(const std::vector<Eigen::Isometry3d>& trajectory_query_,
                                                        const std::vector<Eigen::Isometry3d>& trajectory_reference_);

int32_t main (int32_t argc_, char** argv_) {
  if (argc_ != 3) {
    std::cerr << "usage: ./trajectory_analyzer <trajectory.txt> <messages.txt>" << std::endl;
    return 0;
  }

  //ds determine and log configuration
  const std::string file_name_trajectory_slam         = argv_[1];
  const std::string file_name_trajectory_ground_truth = argv_[2];
  std::cerr << "        trajectory SLAM: " << file_name_trajectory_slam << std::endl;
  std::cerr << "trajectory ground truth: " << file_name_trajectory_ground_truth << std::endl;

  //ds parse SLAM trajectory
  std::ifstream input_stream_trajectory_slam(file_name_trajectory_slam);
  if (!input_stream_trajectory_slam.good() || !input_stream_trajectory_slam.is_open()) {
    std::cerr << "ERROR: unable to open: '" << file_name_trajectory_slam << "'" << std::endl;
    return 0;
  }

  //ds load SLAM trajectory poses
  std::vector<Eigen::Isometry3d> poses_slam(0);
  std::string buffer_line;
  while (std::getline(input_stream_trajectory_slam, buffer_line)) {

    //ds get it to a std::stringstream
    std::istringstream buffer_stream(buffer_line);

    //ds information fields (KITTI format)
    Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
    for (uint8_t u = 0; u < 3; ++u) {
      for (uint8_t v = 0; v < 4; ++v) {
        buffer_stream >> pose(u,v);
      }
    }
    poses_slam.push_back(pose);
  }
  input_stream_trajectory_slam.close();
  std::cerr << "        loaded trajectory SLAM poses: " << poses_slam.size() << std::endl;
  const uint32_t number_of_measurements = poses_slam.size();

  //ds parse ground truth trajectory
  srrg_core::MessageReader message_reader;
  message_reader.open(file_name_trajectory_ground_truth);
  if (!message_reader.good()) {
    std::cerr << "ERROR: unable to open: '" << file_name_trajectory_ground_truth << "'" << std::endl;
    return 0;
  }

  //ds load ground truth poses
  std::vector<Eigen::Isometry3d> poses_ground_truth(0);
  srrg_core::BaseMessage* message = 0;
  while ((message = message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    assert(sensor_message);
    sensor_message->untaint();

    //ds add to synchronizer
    if (sensor_message->topic() == "/camera_left/image_raw") {
      PinholeImageMessage* image_message_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);
      poses_ground_truth.push_back(image_message_left->odometry().cast<double>());
    }
    delete sensor_message;
  }
  message_reader.close();
  std::cerr << "loaded trajectory ground truth poses: " << poses_ground_truth.size() << std::endl;

  //ds check consistency
  if (poses_ground_truth.size() != number_of_measurements) {
    std::cerr << "ERROR: unequal number of SLAM and ground truth measurements (check input files)" << std::endl;
    return 0;
  }

  std::cerr << "\nraw RMSE: " << getAbsoluteTranslationRootMeanSquaredError(poses_slam, poses_ground_truth) << "\n" << std::endl;

  //ds objective
  Eigen::Isometry3d transform_slam_to_ground_truth(Eigen::Isometry3d::Identity());

  //ds ICP configuration
  const uint32_t number_of_iterations = 10;
  const double maximum_error_kernel   = 0.1; //ds (m^2)

  //ds ICP running variables
  Matrix6d H(Matrix6d::Zero());
  Vector6d b(Vector6d::Zero());

  //ds perform least squares optimization
  std::cerr << "optimizing transform .." << std::endl;
  for (uint32_t iteration = 0; iteration < number_of_iterations; ++iteration) {

    //ds initialize setup
    H.setZero();
    b.setZero();
    uint32_t number_of_inliers = 0;
    double total_error_squared = 0;

    //ds for all SLAM trajectory poses
    for (uint32_t index = 0; index < number_of_measurements; ++index) {

      //ds compute current error
      const Eigen::Vector3d& measured_point_in_reference = poses_ground_truth[index].translation();
      const Eigen::Vector3d sampled_point_in_reference   = transform_slam_to_ground_truth*poses_slam[index].translation();
      const Eigen::Vector3d error                        = sampled_point_in_reference-measured_point_in_reference;

      //ds update chi
      const double error_squared = error.transpose()*error;

      //ds check if outlier
      double weight = 1.0;
      if (error_squared > maximum_error_kernel) {
        weight = maximum_error_kernel/error_squared;
      } else {
        ++number_of_inliers;
      }
      total_error_squared += error_squared;

      //ds get the jacobian of the transform part = [I -2*skew(T*modelPoint)]
      Matrix3_6d jacobian;
      jacobian.block<3,3>(0,0).setIdentity();
      jacobian.block<3,3>(0,3) = -2*skew(sampled_point_in_reference);

      //ds precompute transposed
      const Matrix6_3d jacobian_transposed(jacobian.transpose());

      //ds accumulate
      H += weight*jacobian_transposed*jacobian;
      b += weight*jacobian_transposed*error;
    }

    //ds solve the system and update the estimate
    transform_slam_to_ground_truth = srrg_core::v2t(static_cast<const Vector6d&>(H.ldlt().solve(-b)))*transform_slam_to_ground_truth;

    //ds enforce rotation symmetry
    const Eigen::Matrix3d rotation   = transform_slam_to_ground_truth.linear();
    Eigen::Matrix3d rotation_squared = rotation.transpose( )*rotation;
    rotation_squared.diagonal().array()     -= 1;
    transform_slam_to_ground_truth.linear() -= 0.5*rotation*rotation_squared;

    //ds status
    std::printf("iteration: %03u total error (m^2): %9.3f (inliers: %4u/%4u)\n",
                iteration, total_error_squared, number_of_inliers, number_of_measurements);
  }

  //ds compute optimal poses
  std::vector<Eigen::Isometry3d> poses_slam_optimal(poses_slam.size());
  for (uint32_t index = 0; index < number_of_measurements; ++index) {
    poses_slam_optimal[index] = transform_slam_to_ground_truth*poses_slam[index];
  }
  std::cerr << "done" << std::endl;

  //ds done
  std::cerr << "\noptimal RMSE: " << getAbsoluteTranslationRootMeanSquaredError(poses_slam_optimal, poses_ground_truth) << std::endl;
  return 0;
}

//ds computes absolute translation RMSE
const double getAbsoluteTranslationRootMeanSquaredError(const std::vector<Eigen::Isometry3d>& trajectory_query_,
                                                        const std::vector<Eigen::Isometry3d>& trajectory_reference_) {

  if (trajectory_query_.size() != trajectory_reference_.size()) {
    throw std::runtime_error("getAbsoluteTranslationRootMeanSquaredError: ERROR: passed trajectories with unequal number of measurements");
  }

  //ds compute absolute squared errors
  std::vector<double> squared_errors_translation_absolute(0);
  for (uint32_t index = 0; index < trajectory_query_.size(); ++index) {

    //ds compute squared errors between frames
    squared_errors_translation_absolute.push_back((trajectory_query_[index].translation()-trajectory_reference_[index].translation()).squaredNorm());
  }

  //ds compute RMSE
  double root_mean_squared_error_translation_absolute = 0;
  for (const double& squared_error: squared_errors_translation_absolute) {
    root_mean_squared_error_translation_absolute += squared_error;
  }
  root_mean_squared_error_translation_absolute /= squared_errors_translation_absolute.size();
  root_mean_squared_error_translation_absolute = std::sqrt(root_mean_squared_error_translation_absolute);
  squared_errors_translation_absolute.clear();

  //ds done
  return root_mean_squared_error_translation_absolute;
}
