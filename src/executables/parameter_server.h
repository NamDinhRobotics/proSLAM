#pragma once
#include "types/definitions.h"

namespace proslam {

  //ds this class
  class ParameterServer {

    //ds functionality
    public:

      static void parseParametersFromCommandLine(int32_t argc, char ** argv) {
        int32_t number_of_added_parameters = 1;
        while(number_of_added_parameters < argc){
          if (!std::strcmp(argv[number_of_added_parameters], "-camera-left-topic")){
            number_of_added_parameters++;
            _topic_camera_image_left = argv[number_of_added_parameters];
          } else if (!std::strcmp(argv[number_of_added_parameters], "-camera-right-topic")){
            number_of_added_parameters++;
            _topic_camera_image_right = argv[number_of_added_parameters];
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
        if (_topic_camera_image_left.length() == 0) {
          std::cerr << "ERROR: empty value entered for parameter: -camera-left-topic (enter -h for help)" << std::endl;
          exit(0);
        }
        if (_topic_camera_image_right.length() == 0) {
          std::cerr << "ERROR: empty value entered for parameter: -camera-right-topic (enter -h for help)" << std::endl;
          exit(0);
        }

        //ds check dataset length
        if (_filename_dataset.length() == 0) {
          std::cerr << "ERROR: no dataset provided (enter -h for help)" << std::endl;
          exit(0);
        }
      }

      static void printParameters() {
        std::cerr << "-------------------------------------------------------------------------" << std::endl;
        std::cerr << "running with params: " << std::endl;
        std::cerr << "-camera-left-topic  " << _topic_camera_image_left << std::endl;
        std::cerr << "-camera-right-topic " << _topic_camera_image_right << std::endl;
        std::cerr << "-use-gui            " << _option_use_gui << std::endl;
        std::cerr << "-open               " << !_option_use_relocalization << std::endl;
        std::cerr << "-show-top           " << _option_show_top_viewer << std::endl;
        std::cerr << "-drop-framepoints   " << _option_drop_framepoints << std::endl;
        std::cerr << "-equalize-histogram " << _option_equalize_histogram << std::endl;
        std::cerr << "-dataset            " << _filename_dataset << std::endl;
        std::cerr << "-------------------------------------------------------------------------" << std::endl;
      }

    //ds getters/setters
    public:

      static void printBanner() {std::cerr << _banner << std::endl;}
      static const std::string& topicCameraImageLeft() {return _topic_camera_image_left;}
      static const std::string& topicCameraImageRight() {return _topic_camera_image_right;}
      static const std::string& topicCameraInfoLeft() {return _topic_camera_info_left;}
      static const std::string& topicCameraInfoRight() {return _topic_camera_info_right;}
      static const std::string& filenameDataset() {return _filename_dataset;}
      static const bool& optionUseGUI() {return _option_use_gui;}
      static const bool& optionUseRelocalization() {return _option_use_relocalization;}
      static const bool& optionShowTopViewer() {return _option_show_top_viewer;}
      static const bool& optionDropFramepoints() {return _option_drop_framepoints;}
      static const bool& optionEqualizeHistogram() {return _option_equalize_histogram;}

    //ds attributes
    protected:

      //ds informative
      static std::string _banner;

      //ds files/topics
      static std::string _topic_camera_image_left;
      static std::string _topic_camera_image_right;
      static std::string _topic_camera_info_left;
      static std::string _topic_camera_info_right;
      static std::string _filename_dataset;

      //ds flags
      static bool _option_use_gui;
      static bool _option_use_relocalization;
      static bool _option_show_top_viewer;
      static bool _option_drop_framepoints;
      static bool _option_equalize_histogram;
  };

  //ds informative
  std::string ParameterServer::_banner = "-------------------------------------------------------------------------\n"
                                         "srrg_proslam_app: simple SLAM application\n"
                                         "usage: srrg_proslam_app [options] <dataset>\n"
                                         "\n"
                                         "<dataset>: path to a SRRG txt_io dataset file\n"
                                         "\n"
                                         "[options]\n"
                                         "-camera-left-topic <string>:  set left camera topic name (as set in txt_io dataset file)\n"
                                         "-camera-right-topic <string>: set left camera topic name (as set in txt_io dataset file)\n"
                                         "-use-gui (-ug):               displays GUI elements\n"
                                         "-open:                        disables relocalization (open loop mode)\n"
                                         "-show-top (-st):              enable top map viewer\n"
                                         "-drop-framepoints (-df):      deallocation of past framepoints at runtime (reduces memory demand)\n"
                                         "-equalize-histogram (-eh):    equalize stereo image histogram before processing\n"
                                         "-------------------------------------------------------------------------\n";

  //ds files/topics
  std::string ParameterServer::_topic_camera_image_left  = "/camera_left/image_raw";
  std::string ParameterServer::_topic_camera_image_right = "/camera_right/image_raw";
  std::string ParameterServer::_topic_camera_info_left   = "";
  std::string ParameterServer::_topic_camera_info_right  = "";
  std::string ParameterServer::_filename_dataset         = "";

  //ds flags
  bool ParameterServer::_option_use_gui            = false;
  bool ParameterServer::_option_use_relocalization = true;
  bool ParameterServer::_option_show_top_viewer    = false;
  bool ParameterServer::_option_drop_framepoints   = false;
  bool ParameterServer::_option_equalize_histogram = false;
}
