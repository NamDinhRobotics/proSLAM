#include "system/slam_assembly.h"

int32_t main(int32_t argc_, char** argv_) {

  //ds enable opencv2 optimizations
  cv::setUseOptimized(true);

  //ds allocate the complete parameter collection with default values (will be propagated through the complete SLAM system)
  proslam::ParameterCollection* parameters = new proslam::ParameterCollection(proslam::LoggingLevel::Debug);

  try {

    //ds parse parameters from command line (optionally setting the parameter values)
    parameters->parseFromCommandLine(argc_, argv_);
  } catch (const std::runtime_error& exception_) {
    std::cerr << "main|caught exception '" << exception_.what() << "'" << std::endl;
    delete parameters;
    return 0;
  }

  //ds print loaded configuration
  parameters->command_line_parameters->print();

  //ds allocate SLAM system (has internal access to parameter server)
  //ds we allocate it dynamically so we can destroy it when we want
  proslam::SLAMAssembly* slam_system = new proslam::SLAMAssembly(parameters);

  //ds dynamic elements (in this scope for cleanup in case an exception occurs)
  QApplication* gui_server = nullptr;
  std::thread* slam_thread = nullptr;

  try {

    //ds load cameras
    slam_system->loadCamerasFromMessageFile();

    //ds if visualization is desired
    if (parameters->command_line_parameters->option_use_gui) {

      //ds enable two opencv threads
      cv::setNumThreads(2);

      //ds target maximum GUI frequency; 50 fps
      const proslam::real target_display_frequency  = 50;
      const int64_t duration_gui_sleep_milliseconds = 1000/target_display_frequency;

      //ds allocate a qt UI server in the main scope (required)
      gui_server = new QApplication(argc_, argv_);

      //ds initialize GUI
      slam_system->initializeGUI(gui_server);

      //ds start message playback in separate thread
      slam_thread = slam_system->playbackMessageFileInThread();

      //ds enter GUI loop
      while (slam_system->isViewerOpen()) {

        //ds breathe (maximum GUI speed: 50 fps)
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_gui_sleep_milliseconds));

        //ds draw current state
        slam_system->draw();
      }

      //ds clean up GL
      gui_server->closeAllWindows();
      gui_server->quit();

      //ds signal termination request (no effect if processing has already terminated)
      slam_system->requestTermination();

      //ds join system thread
      std::cerr << "main|joining thread: system" << std::endl;
      slam_thread->join();
      delete slam_thread;
      std::cerr << "main|all threads successfully joined" << std::endl;
    } else {

      //ds disable opencv multithreading
      cv::setNumThreads(0);

      //ds plain full-speed message playback in the main thread (blocking)
      slam_system->playbackMessageFile();
    }

    //ds print full report
    slam_system->printReport();

    //ds save trajectories to disk
    slam_system->writeTrajectoryKITTI("trajectory_kitti.txt");
    slam_system->writeTrajectoryTUM("trajectory_tum.txt");

    //ds save g2o graph to disk
    if (parameters->command_line_parameters->option_save_pose_graph) {
      slam_system->writePoseGraphToFile("pose_graph.g2o");
    }
  } catch (const std::runtime_error& exception_) {
    std::cerr << DOUBLE_BAR << std::endl;
    std::cerr << "main|caught runtime exception '" << exception_.what() << "'" << std::endl;
    std::cerr << DOUBLE_BAR << std::endl;

    //ds do not forget to join threads
    slam_thread->join();
    delete slam_thread;
    std::cerr << "main|joining thread: system" << std::endl;
    slam_thread->join();
    delete slam_thread;
    std::cerr << "main|all threads successfully joined" << std::endl;
  }

  //ds clean up dynamic memory
  delete slam_system;
  delete parameters;
  delete gui_server;
  return 0;
}
