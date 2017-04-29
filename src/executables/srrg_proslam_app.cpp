#include "slam_assembly.h"

int32_t main(int32_t argc_, char** argv_) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds allocate the complete parameter collection with default values
  proslam::ParameterCollection* parameters = new proslam::ParameterCollection();

  //ds parse parameters from command line (optionally setting the parameter values)
  parameters->parseFromCommandLine(argc_, argv_);

  //ds print loaded configuration
  parameters->command_line_parameters->print();

  //ds allocate SLAM system (has internal access to parameter server)
  proslam::SLAMAssembly slam_system(parameters);

  //ds initialize system for txt_io
  slam_system.initializeMessageFile();

  //ds load cameras
  slam_system.loadCamerasFromMessageFile();

  //ds allocate a qt UI server in the main scope (required)
  QApplication* ui_server = new QApplication(argc_, argv_);

  //ds initialize gui
  slam_system.initializeGUI(ui_server);

  //ds start message playback - blocks until dataset is completed or aborted
  slam_system.playbackMessageFile();

  //ds print full report
  slam_system.printReport();

  //ds save trajectory to disk
  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds clean up parameters (since not used in GUI)
  delete parameters;

  //ds exit in GUI
  return slam_system.closeGUI();
}
