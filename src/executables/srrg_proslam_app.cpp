#include "slam_assembly.h"

int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration and store it in the global parameter server (singleton)
  proslam::Parameter::parseParametersFromCommandLine(argc, argv);

  //ds print loaded configuration
  proslam::Parameter::printCommandLineParameters();

  //ds allocate SLAM system (has internal access to parameter server)
  proslam::SLAMAssembly slam_system;

  //ds initialize system for txt_io
  slam_system.initializeMessageFile();

  //ds load cameras
  slam_system.loadCamerasFromMessageFile();

  //ds allocate a qt UI server in the main scope (required)
  QApplication* ui_server = new QApplication(argc, argv);

  //ds initialize gui
  slam_system.initializeGUI(ui_server);

  //ds start message playback - blocks until dataset is completed or aborted
  slam_system.playbackMessageFile();

  //ds print full report
  slam_system.printReport();

  //ds save trajectory to disk
  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds exit in GUI
  return slam_system.closeGUI();
}
