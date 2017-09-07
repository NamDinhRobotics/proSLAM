#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

int32_t main(int32_t argc_, char** argv_) {
  if (argc_ != 2) {
    std::cerr << "usage: ./trajectory_converter <pose_graph.g2o>" << std::endl;
    return 0;
  }

  //ds determine and log configuration
  const std::string input_file    = argv_[1];
  const std::string output_file   = "g2o_trajectory.txt";
  const std::string output_format = "KITTI";
  const std::string pose_keyword  = "VERTEX_SE3:QUAT";
  std::cerr << "converting trajectory from raw file: '" << input_file << "'" << std::endl;
  std::cerr << "to trajectory of format '" << output_format << "': '" << output_file << "'" << std::endl;
  std::cerr << "with pose keyword: '" << pose_keyword << "'" << std::endl;

  //ds parse g2o file by hand (avoiding to link against it)
  std::ifstream input_stream(input_file);
  if (!input_stream.good() || !input_stream.is_open()) {
    std::cerr << "ERROR: unable to open: '" << input_file << "'" << std::endl;
    return 0;
  }

  //ds parsing buffers
  std::vector<Eigen::Isometry3d> poses(0);
  std::string line;

  //ds parse the complete input file (assuming continuous, sequential indexing)
  while (std::getline(input_stream, line)) {

    //ds check if the current line contains pose information
    if (line.find(pose_keyword) != std::string::npos) {

      //ds get line to a string stream object
      std::istringstream stringstream(line);

      //ds all possible values
      std::string entry_name;
      uint64_t entry_identifier = 0;
      double translation_x = 0;
      double translation_y = 0;
      double translation_z = 0;
      double quaternion_w  = 0;
      double quaternion_x  = 0;
      double quaternion_y  = 0;
      double quaternion_z  = 0;

      //ds parse the full line and check for failure
      if (!(stringstream >> entry_name >> entry_identifier >> translation_x >> translation_y >> translation_z
                                                           >> quaternion_x >> quaternion_y >> quaternion_z >> quaternion_w)) {
        std::cerr << "ERROR: unable to parse pose lines with keyword: '" << pose_keyword << "'" << std::endl;
        input_stream.close();
        return 0;
      }

      //ds set pose value
      Eigen::Isometry3d parsed_pose(Eigen::Isometry3d::Identity());
      parsed_pose.translation() = Eigen::Vector3d(translation_x, translation_y, translation_z);
      parsed_pose.linear()      = Eigen::Quaterniond(quaternion_w, quaternion_x, quaternion_y, quaternion_z).toRotationMatrix();
      poses.push_back(parsed_pose);
    }
  }
  input_stream.close();
  std::cerr << "parsed poses: " << poses.size() << std::endl;

  //ds open output file (assuming continuous, sequential indexing)
  std::ofstream output_stream(output_file, std::ifstream::out);
  for (const Eigen::Isometry3d& pose: poses) {

    //ds dump transform according to KITTI format
    for (uint8_t u = 0; u < 3; ++u) {
      for (uint8_t v = 0; v < 4; ++v) {
        output_stream << pose(u,v) << " ";
      }
    }
    output_stream << "\n";
  }
  output_stream.close();
  std::cerr << "to trajectory of format '" << output_format << "' to: '" << output_file << "'" << std::endl;
  return 0;
}
