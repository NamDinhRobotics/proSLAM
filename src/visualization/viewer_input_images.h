#pragma once
#include "types/world_map.h"
#include "motion_estimation/base_tracker.h"

namespace proslam {

  class ViewerInputImages{

  public:
    static struct KeyStroke {

#if CV_MAJOR_VERSION == 2

      //ds hardware specific settings
      constexpr static int Escape      = 1048603; //1074790427; //537919515;
      constexpr static int NumpadPlus  = 1114027; //1074855851; //537984939;
      constexpr static int NumpadMinus = 1114029; //1074855853; //537984941;
      constexpr static int Space       = 1048608; //1074790432; //537919520;
      constexpr static int Backspace   = 1113864;
      constexpr static int Num1 = 1048625;
      constexpr static int Num2 = 1048626;
      constexpr static int Num3 = 1048627;
#elif CV_MAJOR_VERSION == 3

      //ds hardware specific settings
      constexpr static int Escape    = 27;
      constexpr static int Space     = 32;
      constexpr static int Backspace = 8;
      constexpr static int Num1 = 1048625;
      constexpr static int Num2 = 1048626;
      constexpr static int Num3 = 1048627;
#else
  #error OpenCV version not supported
#endif

    } KeyStroke;

  public:

    ViewerInputImages(const WorldMap* world_, const std::string& window_name_ = "input: images");
    void initDrawing();
    void drawFeatures();
    void drawFeatureTracking();
    //void drawSolverState();
    const bool updateGUI();
    void switchMode();

  //ds setters/getters
  public:

    void setTracker(const BaseTracker* tracker_) {_tracker = tracker_;}

  protected:
    const WorldMap* _world;
    const BaseTracker* _tracker;
    cv::Mat _current_image;
    Count _cv_wait_key_timeout_milliseconds = 0;
    bool _display_depth_image               = true;
    const std::string _window_name;
  };

}
