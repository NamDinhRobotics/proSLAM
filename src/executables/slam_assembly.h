#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include "map_optimization/graph_optimizer.h"
#include "relocalization/relocalizer.h"
#include "motion_estimation/tracker.h"
#include "visualization/viewer_input_images.h"
#include "visualization/viewer_output_map.h"

namespace proslam {

  //ds simple assembly of the different SLAM modules provided by ProSLAM
  class SLAMAssembly {

    //ds general attributes
    protected:

      //ds the SLAM map, containing landmarks and trajectory
      WorldMap* _world_map;

      //ds pose graph optimization handler
      GraphOptimizer* _optimizer;

      //ds relocalization module
      Relocalizer* _relocalizer;

      //ds tracking component, deriving the robots odometry
      Tracker* _tracker;

      //ds Qt UI server
      QApplication* _ui_server = 0;

      //ds raw image input processing viewer
      ViewerInputImages* _tracker_viewer = 0;

      //ds 3D output viewers
      ViewerOutputMap* _context_viewer_bird = 0;
      ViewerOutputMap* _context_viewer_top  = 0;

    //ds txt_io playback components
    protected:

      srrg_core::MessageTimestampSynchronizer _synchronizer;

      srrg_core::MessageReader _sensor_message_reader;

      std::vector<std::string> _camera_topics_synchronized;

      CameraMap _cameras_by_topic;


  };
}
