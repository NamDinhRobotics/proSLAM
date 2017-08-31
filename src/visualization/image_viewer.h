#pragma once
#include <mutex>
#include "types/frame.h"

namespace proslam {

//! @class a simple 2D opencv input image viewer class for processing display
class ImageViewer {

//ds object life
public:

  //! @brief default constructor
  //! @brief param[in] parameters_ viewer parameters
  //! @brief param[in] window_name_ target window title of the OpenCV window
  ImageViewer(const ImageViewerParameters* parameters_,
              const std::string& window_title_ = "input: images [OpenCV]");

  //! @brief default destructor
  ~ImageViewer();

//ds access
public:

  //! @brief GUI update function, copies framepoints from provided frame into thread-safe, drawable containers  - LOCKING
  //! @param[in] frame_ the frame to display
  void update(const Frame* frame_);

  //! @brief draw function - LOCKING
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

  //! @brief current frame handle (drawn in draw function)
  const Frame* _current_frame;

  //! @brief currently displayed image
  cv::Mat _current_image;

//ds class specific
private:

  //! @brief configurable parameters
  const ImageViewerParameters* _parameters;
};
}
