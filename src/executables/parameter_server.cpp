#include "parameter_server.h"

namespace proslam {

  //ds informative
  std::string ParameterServer::_banner = "-------------------------------------------------------------------------\n"
                                         "srrg_proslam_app: simple SLAM application\n"
                                         "usage: srrg_proslam_app [options] <dataset>\n"
                                         "\n"
                                         "<dataset>: path to a SRRG txt_io dataset file\n"
                                         "\n"
                                         "[options]\n"
                                         "-topic-image-left (-il)        <string>: sets left image topic name (txt_io, ROS)\n"
                                         "-topic-image-right (-ir)       <string>: sets right image topic name (txt_io, ROS)\n"
                                         "-topic-camera-info-left (-cl)  <string>: sets left camera info topic (ROS)\n"
                                         "-topic-camera-info-right (-cr) <string>: sets right camera info topic (ROS)\n"
                                         "-use-gui (-ug):                          displays GUI elements\n"
                                         "-open:                                   disables relocalization (open loop mode)\n"
                                         "-show-top (-st):                         enable top map viewer\n"
                                         "-drop-framepoints (-df):                 deallocation of past framepoints at runtime (reduces memory demand)\n"
                                         "-equalize-histogram (-eh):               equalize stereo image histogram before processing\n"
                                         "-rectify-and-undistort (-ru):            rectifies and undistorts input images based on camera info\n"
                                         "-------------------------------------------------------------------------";

  //ds files/topics
  std::string ParameterServer::_topic_image_left         = "/camera_left/image_raw";
  std::string ParameterServer::_topic_image_right        = "/camera_right/image_raw";
  std::string ParameterServer::_topic_camera_info_left   = "";
  std::string ParameterServer::_topic_camera_info_right  = "";
  std::string ParameterServer::_filename_dataset         = "";

  //ds flags
  bool ParameterServer::_option_use_gui               = false;
  bool ParameterServer::_option_use_relocalization    = true;
  bool ParameterServer::_option_show_top_viewer       = false;
  bool ParameterServer::_option_drop_framepoints      = false;
  bool ParameterServer::_option_equalize_histogram    = false;
  bool ParameterServer::_option_rectify_and_undistort = false;
}
