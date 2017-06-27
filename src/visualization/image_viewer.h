#pragma once
#include <mutex>
#include "types/frame.h"

namespace proslam {

//! @class a simple 2D opencv input image viewer class for processing display
class ImageViewer {

//ds exports
public:

  //! @struct thread-safe, drawable container for a FramePoint object
  struct DrawableFramePoint {
    PointCoordinates image_coordinates;
    PointCoordinates reprojection_coordinates;
    bool has_landmark;
    bool is_landmark_near;
    bool is_valid;
    bool has_previous;
    PointCoordinates image_coordinates_previous;
    bool is_merged;
    real color_intensity;
  };

//ds object life
public:

  //! @brief default constructor
  //! @brief param[in] window_name_ target window title of the OpenCV window
  ImageViewer(const std::string& window_title_ = "input: images");

  //! @brief default destructor
  ~ImageViewer();

//ds access
public:

  //! @brief GUI update function, copies framepoints from provided frame into thread-safe, drawable containers
  //! @param[in] frame_ the frame to display
  void update(const Frame* frame_);

  //! @brief draw function
  void draw();

//ds helpers
protected:

  //! @brief draws currently generated framepoints with image coordinates
  void _drawPoints();

  //! @brief draws the currently tracked framepoints epipolar lines (< generated)
  void _drawTracking();

//ds attributes
protected:

  //! @brief viewer window title
  const std::string _window_title;

  //! @brief mutex for data exchange, owned by the viewer
  std::mutex _mutex_data_exchange;

  //! @brief active framepoint vector copy from tracker (updated with update method)
  std::vector<DrawableFramePoint> _active_framepoints;

  //! @brief currently displayed image
  cv::Mat _current_image;
};
}
