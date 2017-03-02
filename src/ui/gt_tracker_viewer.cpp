#include "gt_tracker_viewer.h"

using namespace std;

namespace gslam {

  TrackerViewer::TrackerViewer(WorldContext* world_): _world(world_), _cv_wait_key_timeout_milliseconds(0) {
    std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
  }
  
  void TrackerViewer::initDrawing(){
    if (!_world) {
      return;
    }
    if (!_world->currentTrackingContext()) {
      return;
    }
    const Frame* current_frame = _world->currentTrackingContext()->currentFrame();
    if(! current_frame) {
      return;
    }

    //ds if at least one itensity image is available
    if (current_frame->hasIntensity()) {

      //ds if depth display is desired and available
      if (_display_depth_image && current_frame->hasDepth()) {

        //ds display rgbd images in a stereo frmae
        cv::Mat image_left;
        cv::cvtColor(current_frame->intensityImage(), image_left, CV_GRAY2RGB);
        cv::Mat image_right = current_frame->depthImage();

        //ds obtain depth scaling and get it to rgb
        double depth_minimum;
        double depth_maximum;
        cv::minMaxIdx(image_right, &depth_minimum, &depth_maximum);
        cv::Mat _image_depth_scaled;
        cv::convertScaleAbs(image_right, _image_depth_scaled, 255.0/depth_maximum);
        cv::cvtColor(_image_depth_scaled, image_right, CV_GRAY2RGB);

        //ds create stereo image
        cv::hconcat(image_left, image_right, _current_image);
      } else if (current_frame->hasIntensityExtra()) {

        //ds get rgb copies of both images
        cv::Mat image_left_rgb;
        cv::Mat image_right_rgb;
        cv::cvtColor(current_frame->intensityImage(), image_left_rgb, CV_GRAY2RGB);
        cv::cvtColor(current_frame->intensityImageExtra(), image_right_rgb, CV_GRAY2RGB);

        //ds create stereo image
        cv::hconcat(image_left_rgb, image_right_rgb, _current_image);
      } else {

        //ds simple rgb display
        cv::cvtColor(current_frame->intensityImage(), _current_image, CV_GRAY2RGB);
      }
    }
  }

  void TrackerViewer::drawFeatures(){

    //ds check if we have an active frame
    const Frame* current_frame = _world->currentTrackingContext()->currentFrame();
    if (! current_frame) {
      return;
    }

    //ds for all points in the current frame
    for (const FramePoint* point: current_frame->points()) {
      cv::Scalar color = CV_COLOR_CODE_WHITE;

      //ds if the point is representing a landmark view
      if (point->landmark()) {

        //ds compute intensity -> old landmarks appear more bright
        //const gt_real intensity = std::min(static_cast<gt_real>(1.0), point->age()/static_cast<gt_real>(50));
        const gt_real intensity = std::min(static_cast<gt_real>(1.0), point->age()/static_cast<gt_real>(25)); //ds KITTI

        //ds check validity
        if (point->landmark()->isValidated()) {

          //ds check landmark kind: by vision or by depth
          if (point->landmark()->isByVision()) {
            color = cv::Scalar(100+intensity*155, 0, 100+intensity*155);
          } else {
            color = cv::Scalar(100+intensity*155, 0, 0);
          }
        } else {
          color = CV_COLOR_CODE_RED;
        }

        if (point->landmark()->isInLoopClosureQuery() || point->landmark()->isInLoopClosureReference()) {
          color = CV_COLOR_CODE_DARKGREEN;
        } else if (point->landmark()->isInMapMergeQuery() || point->landmark()->isInMapMergeReference()) {
          color = CV_COLOR_CODE_BROWN;
        }

        //ds draw reprojection - if set
        if (point->reprojectionCoordinates().x() > 0 || point->reprojectionCoordinates().y() > 0) {
          cv::circle(_current_image, cv::Point(point->reprojectionCoordinates().x(), point->reprojectionCoordinates().y()), 4, color);

          if (current_frame->hasIntensityExtra()) {
            cv::circle(_current_image, cv::Point(point->reprojectionCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->reprojectionCoordinatesExtra().y()), 4, color);
          }
        } else {
//          cv::circle(_current_image, cv::Point(point->imageCoordinates().x(), point->imageCoordinates().y()), 4, CV_COLOR_CODE_RED);
//
//          if (current_frame->hasIntensityExtra()) {
//            cv::circle(_current_image, cv::Point(point->imageCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->imageCoordinatesExtra().y()), 4, CV_COLOR_CODE_RED);
//          }
        }
      } else {
        switch(point->status()){
          case FramePoint::Created:
            continue; //color = CV_COLOR_CODE_GREEN; break;
          case FramePoint::Confirmed:
            continue; //color = CV_COLOR_CODE_ORANGE; break;
          case FramePoint::Persistent:
            continue; //color = CV_COLOR_CODE_RED; break;
          default:
            color = CV_COLOR_CODE_WHITE; break;
        }
      }

      //ds draw the point
      cv::circle(_current_image, cv::Point(point->imageCoordinates().x(), point->imageCoordinates().y()), 2, color, -1);

      if (current_frame->hasIntensityExtra()) {
        cv::circle(_current_image, cv::Point(point->imageCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->imageCoordinatesExtra().y()), 2, color, -1);
      }

      //ds draw point index if desired
      if (_display_point_index && point->status() == FramePoint::Confirmed) {

        //ds buffer id and draw it on the image
        char buffer[10];
        std::snprintf(buffer, 10, "%lu", point->index());
        cv::putText(_current_image, buffer, cv::Point2i(point->imageCoordinates().x()+5, point->imageCoordinates().y()+5), cv::FONT_HERSHEY_PLAIN, 0.5, CV_COLOR_CODE_RED);
      }
    }
  }

  void TrackerViewer::drawFeatureTracking(){
    const Frame* current_frame=_world->currentTrackingContext()->currentFrame();
    if (! current_frame)
      return;
    for(size_t i=0; i<current_frame->points().size(); i++){
      const FramePoint* current_point=current_frame->points()[i];
      const FramePoint* parent_point_ptr=current_point->previous();
      if(! parent_point_ptr)
	continue;
      const FramePoint& parent_point=*parent_point_ptr;
      cv::Point current_pt(current_point->imageCoordinates().x(),
			   current_point->imageCoordinates().y());
      cv::Point previous_pt(parent_point.imageCoordinates().x(),
			    parent_point.imageCoordinates().y());
      cv::line(_current_image, current_pt,previous_pt, CV_COLOR_CODE_GREEN);
    }
  }



  // void LKTrackerViewer::drawSolverState(){
  //   cv::Scalar color(0,0,255);
  //   for(size_t i=0; i<_current_image_info.points.size(); i++){
  //     const PointTrack& point=_current_image_info.points[i];
  //     if (point.status&PointTrack::ConfirmedFromPosit) {
  // 	if (point.reprojected_coordinates.z()<=0)
  // 	  continue;
  // 	cv::Point reprojected_pt(point.reprojected_coordinates.x(), 
  // 				 point.reprojected_coordinates.y());
  // 	cv::Point image_pt(point.image_coordinates.x(), 
  // 			   point.image_coordinates.y());
  // 	Eigen::Vector3f dp=point.reprojected_coordinates-point.image_coordinates;
  // 	cv::circle(_current_image,reprojected_pt, 3, color);
  // 	cv::line(_current_image, reprojected_pt, image_pt, color); 
  //     }
  //   }
  // }

  const bool TrackerViewer::updateGUI(){
    if (!_current_image.empty()) {
      cv::imshow("TrackerViewer", _current_image);

      int last_key_stroke = cv::waitKey(_cv_wait_key_timeout_milliseconds);
      if(last_key_stroke != -1) {
        switch(last_key_stroke) {

          case KeyStroke::Escape: {
            std::cerr << "termination requested" << std::endl;
            return false;
          }

          case KeyStroke::Backspace: {
            if(_cv_wait_key_timeout_milliseconds > 0) {
              _cv_wait_key_timeout_milliseconds = 0;
              std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
            }
            else {
              _cv_wait_key_timeout_milliseconds = 1;
              std::cerr << "switched to benchmark mode (press backspace for switch)" << std::endl;
            }
            break;
          }

          case KeyStroke::Num1: {
            _display_depth_image = !_display_depth_image;
            break;
          }

          case KeyStroke::Num2: {
            _display_point_index = !_display_point_index;
            break;
          }

          default:{
            break;
          }
        }
      }
    }

    return true;
  }

  void TrackerViewer::switchToStepwiseMode() {
    _cv_wait_key_timeout_milliseconds = 0;
    std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
  }
}
