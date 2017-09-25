#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

#include "srrg_types/types.hpp"

using namespace srrg_core;

struct PositionMeasurement {
    PositionMeasurement(const double& timestamp_seconds_,
                        const Eigen::Vector3d& position_): timestamp_seconds(timestamp_seconds_),
                                                           position(position_) {}
    PositionMeasurement(): timestamp_seconds(0), position(Eigen::Vector3d::Zero()) {}
    double timestamp_seconds;
    Eigen::Vector3d position;
};

const double getAbsoluteTranslationRootMeanSquaredError(const std::vector<std::pair<PositionMeasurement, PositionMeasurement>> position_correspondences_);

const Eigen::Vector3d getInterpolatedPositionLinear(const PositionMeasurement& ground_truth_previous_,
                                                    const PositionMeasurement& ground_truth_next_,
                                                    const PositionMeasurement& measurement_);

int32_t main (int32_t argc_, char** argv_) {
  if (argc_ < 5) {
    std::cerr << "usage: ./trajectory_analyzer -tum <trajectory.txt> -asl <ground_truth.txt> [-skip <integer>]" << std::endl;
    return 0;
  }

  //ds determine and log configuration
  std::string file_name_trajectory_slam = argv_[1];
  std::string file_name_trajectory_ground_truth = argv_[2];
  uint32_t number_of_poses_to_skip = 0;

  //ds parse configuration
  int32_t number_of_checked_parameters = 1;
  while (number_of_checked_parameters < argc_) {
    if (!std::strcmp(argv_[number_of_checked_parameters], "-tum")) {
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      file_name_trajectory_slam = argv_[number_of_checked_parameters];
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-asl")) {
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      file_name_trajectory_ground_truth = argv_[number_of_checked_parameters];
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-skip")) {
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      number_of_poses_to_skip = std::stoi(argv_[number_of_checked_parameters]);
    }
    ++number_of_checked_parameters;
  }

  //ds log configuration
  std::cerr << "file_name_trajectory_slam: " << file_name_trajectory_slam << std::endl;
  std::cerr << "file_name_trajectory_ground_truth: " << file_name_trajectory_ground_truth << std::endl;
  std::cerr << "number_of_poses_to_skip: " << number_of_poses_to_skip << std::endl;

  //ds parse SLAM trajectory
  std::ifstream input_stream_trajectory_slam(file_name_trajectory_slam);
  if (!input_stream_trajectory_slam.good() || !input_stream_trajectory_slam.is_open()) {
    std::cerr << "ERROR: unable to open: '" << file_name_trajectory_slam << "'" << std::endl;
    return 0;
  }

  //ds load SLAM trajectory poses
  std::vector<PositionMeasurement> positions_slam;
  std::string buffer_line;
  uint32_t skipped_poses = 0;
  while (std::getline(input_stream_trajectory_slam, buffer_line)) {

    //ds get line to a string stream object
    std::istringstream stringstream(buffer_line);

    //ds possible values
    double timestamp_seconds = 0;
    double translation_x = 0;
    double translation_y = 0;
    double translation_z = 0;
    double quaternion_w  = 0;
    double quaternion_x  = 0;
    double quaternion_y  = 0;
    double quaternion_z  = 0;

    //ds parse the full line and check for failure
    if (!(stringstream >> timestamp_seconds >> translation_x >> translation_y >> translation_z
                                            >> quaternion_x >> quaternion_y >> quaternion_z >> quaternion_w)) {
      std::cerr << "ERROR: unable to parse pose lines" << std::endl;
      input_stream_trajectory_slam.close();
      return 0;
    }

    //ds set pose value
    Eigen::Isometry3d parsed_pose(Eigen::Isometry3d::Identity());
    parsed_pose.translation() = Eigen::Vector3d(translation_x, translation_y, translation_z);
    parsed_pose.linear()      = Eigen::Quaterniond(quaternion_w, quaternion_x, quaternion_y, quaternion_z).toRotationMatrix();

    if (skipped_poses >= number_of_poses_to_skip) {
      positions_slam.push_back(PositionMeasurement(timestamp_seconds, parsed_pose.translation()));
    } else {
      ++skipped_poses;
    }
  }
  input_stream_trajectory_slam.close();

  //ds check skip
  if (number_of_poses_to_skip >= positions_slam.size()) {
    std::cerr << "ERROR: insufficient number of measurements for number_of_poses_to_skip: " << number_of_poses_to_skip << std::endl;
    return 0;
  }

  //ds also cut skipped poses from the end
  positions_slam.resize(positions_slam.size()-number_of_poses_to_skip);
  std::cerr << "loaded trajectory SLAM positions: " << positions_slam.size() << " for: " << file_name_trajectory_slam << std::endl;

  //ds parse ground truth trajectory
  std::ifstream input_stream_trajectory_ground_truth(file_name_trajectory_ground_truth);
  if (!input_stream_trajectory_ground_truth.good() || !input_stream_trajectory_ground_truth.is_open()) {
    std::cerr << "ERROR: unable to open: '" << file_name_trajectory_ground_truth << "'" << std::endl;
    return 0;
  }

  //ds load ground truth poses
  std::vector<PositionMeasurement> positions_ground_truth;
  while (std::getline(input_stream_trajectory_ground_truth, buffer_line)) {

    //ds skip comment lines
    if (buffer_line[0] == '#') {
      continue;
    }

    //ds parse control
    std::string::size_type index_begin_item = 0;
    std::string::size_type index_end_item   = 0;

    //ds parse timestamp
    index_end_item = buffer_line.find(",", index_begin_item);
    const uint64_t timestamp_nanoseconds = std::atol(buffer_line.substr(index_begin_item, index_end_item).c_str());
    const double timestamp_seconds       = timestamp_nanoseconds/1e9;
    index_begin_item = index_end_item+1;

    //ds position buffer
    Eigen::Isometry3d leica_to_world(Eigen::Isometry3d::Identity());
    for (uint32_t row = 0; row < 3; ++row) {
      index_end_item = buffer_line.find(",", index_begin_item);
      leica_to_world.translation()(row) = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
      index_begin_item = index_end_item+1;
    }

    //ds add to buffer
    positions_ground_truth.push_back(PositionMeasurement(timestamp_seconds, leica_to_world.translation()));
  }
  input_stream_trajectory_ground_truth.close();
  std::cerr << "loaded trajectory ground truth positions: " << positions_ground_truth.size() << " for: " << file_name_trajectory_ground_truth << std::endl;

  //ds corresponding measurements
  std::vector<std::pair<PositionMeasurement, PositionMeasurement>> position_correspondences;
  Eigen::Vector3d position_shift(Eigen::Vector3d::Zero());

  //ds for each measurement
  for (uint64_t index_slam = 0; index_slam < positions_slam.size(); ++index_slam) {
    PositionMeasurement& measurement = positions_slam[index_slam];

    //ds find closest ground truth point - bruteforce
    double timestamp_difference_seconds_best = 1;
    uint32_t index_best                      = 0;
    for (uint64_t index_ground_truth = 0; index_ground_truth < positions_ground_truth.size(); ++index_ground_truth) {
      const double timestamp_difference_seconds = std::fabs(measurement.timestamp_seconds-positions_ground_truth[index_ground_truth].timestamp_seconds);
      if (timestamp_difference_seconds < timestamp_difference_seconds_best) {
        timestamp_difference_seconds_best = timestamp_difference_seconds;
        index_best = index_ground_truth;
      }
    }

    //ds skip until we arrive at the ground truth timestamp
    if (index_best == 0) {
      continue;
    }

    //ds solution
    PositionMeasurement ground_truth_interpolated(measurement.timestamp_seconds, measurement.position);

    //ds interpolation: check if before the system
    if (positions_ground_truth[index_best].timestamp_seconds < measurement.timestamp_seconds) {

      //ds interpolate to next
      ground_truth_interpolated.position = getInterpolatedPositionLinear(positions_ground_truth[index_best], positions_ground_truth[index_best+1], measurement);
    } else {

      //ds interpolate from previous
      ground_truth_interpolated.position = getInterpolatedPositionLinear(positions_ground_truth[index_best-1], positions_ground_truth[index_best], measurement);
    }

    //ds for the first measurement - compute starting point offset
    if (index_slam == 0) {
      position_shift = ground_truth_interpolated.position;
    }

    //ds adjust position
    measurement.position += position_shift;

    //ds save the first correspondence
    position_correspondences.push_back(std::make_pair(measurement, ground_truth_interpolated));
  }
  std::cerr << "\ninterpolated positions: " << position_correspondences.size() << " for: " << file_name_trajectory_ground_truth << std::endl;

  std::cerr << "\nraw RMSE: " << getAbsoluteTranslationRootMeanSquaredError(position_correspondences) << "\n" << std::endl;

  //ds objective
  Eigen::Isometry3d transform_slam_to_ground_truth(Eigen::Isometry3d::Identity());

  //ds ICP configuration
  const uint32_t number_of_iterations = 100;
  const double maximum_error_kernel   = 1; //ds (m^2)

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
    for (const std::pair<PositionMeasurement, PositionMeasurement>& position_correspondence: position_correspondences) {

      //ds compute current error
      const Eigen::Vector3d& measured_point_in_reference = position_correspondence.second.position;
      const Eigen::Vector3d sampled_point_in_reference   = transform_slam_to_ground_truth*position_correspondence.first.position;
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
    std::printf("iteration: %03u total error (m^2): %12.3f (inliers: %4u/%4lu=%4.2f)\n",
                iteration, total_error_squared, number_of_inliers, position_correspondences.size(), static_cast<double>(number_of_inliers)/position_correspondences.size());
  }

  //ds compute optimal poses
  for (std::pair<PositionMeasurement, PositionMeasurement>& position_correspondence: position_correspondences) {
    position_correspondence.first.position = transform_slam_to_ground_truth*position_correspondence.first.position;
  }
  std::cerr << "done" << std::endl;

  //ds done
  std::cerr << "\noptimal RMSE: " << getAbsoluteTranslationRootMeanSquaredError(position_correspondences) << std::endl;
  return 0;
}

const double getAbsoluteTranslationRootMeanSquaredError(const std::vector<std::pair<PositionMeasurement, PositionMeasurement>> position_correspondences_) {

  //ds compute absolute squared errors
  std::vector<double> squared_errors_translation_absolute(0);
  for (const std::pair<PositionMeasurement, PositionMeasurement>& position_correspondence: position_correspondences_) {

    //ds compute squared errors between frames
    squared_errors_translation_absolute.push_back((position_correspondence.first.position-position_correspondence.second.position).squaredNorm());
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

const Eigen::Vector3d getInterpolatedPositionLinear(const PositionMeasurement& ground_truth_previous_,
                                              const PositionMeasurement& ground_truth_next_,
                                              const PositionMeasurement& measurement_) {

  //ds interpolate to next
  const double timestamp_difference_seconds_ground_truth = ground_truth_next_.timestamp_seconds-ground_truth_previous_.timestamp_seconds;
  const double timestamp_difference_seconds = measurement_.timestamp_seconds-ground_truth_previous_.timestamp_seconds;
//  std::printf("interpolating timestamps: [%f, %f, %f\n", ground_truth_previous_.timestamp_seconds,
//                                                         measurement_.timestamp_seconds,
//                                                         ground_truth_next_.timestamp_seconds);

  //ds compute interpolated reference measurement
  const Eigen::Vector3d position_interpolated = ground_truth_previous_.position +
                                                timestamp_difference_seconds/timestamp_difference_seconds_ground_truth*
                                                (ground_truth_next_.position-ground_truth_previous_.position);

//  std::cerr << ground_truth_previous_.position.transpose() << std::endl;
//  std::cerr << position_interpolated.transpose() << std::endl;
//  std::cerr << ground_truth_next_.position.transpose() << std::endl;

  return position_interpolated;
}
