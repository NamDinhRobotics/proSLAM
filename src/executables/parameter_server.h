#pragma once
#include "types/definitions.h"

namespace proslam {

  //ds this class serves as a singleton parameter base for the ProSLAM pipeline
  class ParameterServer {
    
    //ds functionality
    public:
    enum TrackerMode {Stereo=0x1, Depth=0x2};

      static void parseParametersFromCommandLine(int32_t argc, char ** argv) {
        int32_t number_of_added_parameters = 1;
        while(number_of_added_parameters < argc){
          if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-left") || !std::strcmp(argv[number_of_added_parameters], "-il")){
            number_of_added_parameters++;
            _topic_image_left = argv[number_of_added_parameters];
          } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-right") || !std::strcmp(argv[number_of_added_parameters], "-ir")){
            number_of_added_parameters++;
            _topic_image_right = argv[number_of_added_parameters];
          } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-left") || !std::strcmp(argv[number_of_added_parameters], "-cl")){
            number_of_added_parameters++;
            _topic_camera_info_left = argv[number_of_added_parameters];
          } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-right") || !std::strcmp(argv[number_of_added_parameters], "-cr")){
            number_of_added_parameters++;
            _topic_camera_info_right = argv[number_of_added_parameters];
          } else if (!std::strcmp(argv[number_of_added_parameters], "-h") || !std::strcmp(argv[number_of_added_parameters], "--h")) {
            printBanner();
            exit(0);
          } else if (!std::strcmp(argv[number_of_added_parameters], "-use-gui") || !std::strcmp(argv[number_of_added_parameters], "-ug")) {
            _option_use_gui = true;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-open")) {
            _option_use_relocalization = false;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-show-top") || !std::strcmp(argv[number_of_added_parameters], "-st")) {
            _option_show_top_viewer = true;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-drop-framepoints") || !std::strcmp(argv[number_of_added_parameters], "-df")) {
            _option_drop_framepoints = true;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-equalize-histogram") || !std::strcmp(argv[number_of_added_parameters], "-eh")) {
            _option_equalize_histogram = true;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-rectify-and-undistort") || !std::strcmp(argv[number_of_added_parameters], "-ru")) {
            _option_rectify_and_undistort = true;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-depth-mode") || !std::strcmp(argv[number_of_added_parameters], "-dm")) {
            _tracker_mode = Depth;
          } else if (!std::strcmp(argv[number_of_added_parameters], "-use-odometry") || !std::strcmp(argv[number_of_added_parameters], "-uo")) {
            _option_use_odometry = true;
          } else {
            _filename_dataset = argv[number_of_added_parameters];
          }
          number_of_added_parameters++;
        }

        //ds validate input parameters and exit on failure
        validateParameters();
      }

      static void validateParameters() {

        //ds check camera topics
        if (_topic_image_left.length() == 0) {
          std::cerr << "ERROR: empty value entered for parameter: -topic-image-left (-il) (enter -h for help)" << std::endl;
          exit(0);
        }
        if (_topic_image_right.length() == 0) {
          std::cerr << "ERROR: empty value entered for parameter: -topic-image-right (-ir) (enter -h for help)" << std::endl;
          exit(0);
        }
      }

      static void printParameters() {
        std::cerr << "-------------------------------------------------------------------------" << std::endl;
        std::cerr << "running with params:" << std::endl;
        std::cerr << "-topic-image-left        " << _topic_image_left << std::endl;
        std::cerr << "-topic-image-right       " << _topic_image_right << std::endl;
        if (_topic_camera_info_left.length() > 0) std::cerr << "-topic-camera-left-info  " << _topic_camera_info_left << std::endl;
        if (_topic_camera_info_right.length() > 0) std::cerr << "-topic-camera-right-info " << _topic_camera_info_right << std::endl;
        std::cerr << "-use-gui                 " << _option_use_gui << std::endl;
        std::cerr << "-open                    " << !_option_use_relocalization << std::endl;
        std::cerr << "-show-top                " << _option_show_top_viewer << std::endl;
        std::cerr << "-use-odometry            " << _option_use_odometry << std::endl;
        std::cerr << "-depth-mode              " << (_tracker_mode==TrackerMode::Depth) << std::endl;
        std::cerr << "-drop-framepoints        " << _option_drop_framepoints << std::endl;
        std::cerr << "-equalize-histogram      " << _option_equalize_histogram << std::endl;
        std::cerr << "-rectify-and-undistort   " << _option_rectify_and_undistort << std::endl;
        if (_filename_dataset.length() > 0) std::cerr << "-dataset                 " << _filename_dataset << std::endl;
        std::cerr << "-------------------------------------------------------------------------" << std::endl;
      }

    //ds getters/setters
    public:

      static void printBanner() {std::cerr << _banner << std::endl;}
      static const std::string& topicImageLeft() {return _topic_image_left;}
      static const std::string& topicImageRight() {return _topic_image_right;}
      static const std::string& topicCameraInfoLeft() {return _topic_camera_info_left;}
      static const std::string& topicCameraInfoRight() {return _topic_camera_info_right;}
      static const std::string& filenameDataset() {return _filename_dataset;}
      static const bool& optionUseGUI() {return _option_use_gui;}
      static const bool& optionUseRelocalization() {return _option_use_relocalization;}
      static const bool& optionShowTopViewer() {return _option_show_top_viewer;}
      static const bool& optionDropFramepoints() {return _option_drop_framepoints;}
      static const bool& optionEqualizeHistogram() {return _option_equalize_histogram;}
      static const bool& optionRectifyAndUndistort() {return _option_rectify_and_undistort;}
      static const bool& optionUseOdometry() {return _option_use_odometry;}
      static const TrackerMode trackerMode() {return _tracker_mode;}
      
    //ds attributes
    protected:

      //ds informative
      static std::string _banner;

      //ds files/topics
      static std::string _topic_image_left;
      static std::string _topic_image_right;
      static std::string _topic_camera_info_left;
      static std::string _topic_camera_info_right;
      static std::string _filename_dataset;

      //ds options
      static bool _option_use_gui;
      static bool _option_use_relocalization;
      static bool _option_show_top_viewer;
      static bool _option_drop_framepoints;
      static bool _option_equalize_histogram;
      static bool _option_rectify_and_undistort;
      static bool _option_use_odometry;
      static TrackerMode _tracker_mode;
  };
}
