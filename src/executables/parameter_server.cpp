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
  std::string ParameterServer::_topic_camera_info_left   = "/camera_left/camera_info";
  std::string ParameterServer::_topic_camera_info_right  = "/camera_right/camera_info";
  std::string ParameterServer::_filename_dataset         = "";

  //ds flags
  bool ParameterServer::_option_use_gui            = false;
  bool ParameterServer::_option_use_relocalization = true;
  bool ParameterServer::_option_show_top_viewer    = false;
  bool ParameterServer::_option_drop_framepoints   = false;
  bool ParameterServer::_option_equalize_histogram = false;
}
