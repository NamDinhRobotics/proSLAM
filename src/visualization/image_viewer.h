#pragma once
#include <mutex>
#include "types/frame.h"

namespace proslam {

//! @class a simple 2D opencv input image viewer class for processing display
class ImageViewer {

//ds object life
PROSLAM_MAKE_PROCESSING_CLASS(ImageViewer)

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

  //! @brief mutex for data exchange, owned by the viewer
  std::mutex _mutex_data_exchange;

  //! @brief current frame handle (drawn in draw function)
  const Frame* _current_frame;

  //! @brief currently displayed image
  cv::Mat _current_image;
};
}
