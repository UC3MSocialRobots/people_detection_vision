#ifndef FAST_CLUSTER_FUNCTIONS_H
#define FAST_CLUSTER_FUNCTIONS_H

// ros
#include <ros/ros.h>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// vision
#include "image_utils/drawing_utils.h"
#include "color/color_utils.h"

namespace fast_cluster_functions {

//! the index used to indicate no object is tracked
static const int NO_OBJECT = -1;

//! the topic where we emit the GUI image
static const std::string GUI_TOPIC =
    "fast_cluster_detector_img_gui";

/*! the topic where we emit the image containing the objects name.
  The images then need to be converted to visible colors. */
static const std::string OBJECTS_NAMES_IMAGE_TOPIC =
    "fast_cluster_detector_objects_name_img";

//! the topic where we emit the image with the objects color
static const std::string COMPONENTS_ILLUS_TOPIC =
    "fast_cluster_detector_components_illus";

//! the topic where we emit the number of objects in the detector
static const std::string OBJECTS_NB_TOPIC =
    "fast_cluster_detector_objects_nb";

//! the topic where we emit the name of the tracked object
static const std::string TRACKED_OBJECT_COORDINATES_TOPIC =
    "fast_cluster_detector_tracked_object_name";

//! the topic where we emit the 3D position of the tracked object
static const std::string TRACKED_OBJECT_POSE_TOPIC =
    "fast_cluster_detector_moving_goal";

//! the color of the tracked object on the illustration frame
static const cv::Scalar TRACKED_OBJECT_COLOR(255, 0, 0); // red

/*!
 * Paint the object image into visible colors
 * \param objects_names_img
 *    The input image containing the names of the objects
 * \param components_illus
 *    OUT the generated image
 * \param tracked_obj_name
 *    if different from NO_OBJECT, will paint with \a TRACKED_OBJECT_COLOR
 *    the object corresponding to that name
 * \param scale
 *    1 for no rescale of the image
 * \param resized_components_illus
 *    if \a scale != 1, a pointer to a resized version of \a components_illus
 */
inline void paint_object_image(const cv::Mat1i & objects_names_img,
                               cv::Mat3b & components_illus,
                               const int & tracked_obj_name = NO_OBJECT,
                               const double scale = 1,
                               cv::Mat3b* resized_components_illus = NULL) {
  components_illus.create(objects_names_img.size());
  components_illus.setTo(0);
  const int* objects_names_img_ptr = &(objects_names_img(0));
  cv::Vec3b* components_illus_ptr = &(components_illus(0));
  int ncols = objects_names_img.cols;

  int n_values = objects_names_img.cols * objects_names_img.rows;
  for (int value_idx = 0; value_idx < n_values; ++value_idx) {
    // paint the pixel with the correct color
    if (*objects_names_img_ptr != NO_OBJECT) {
      if (*objects_names_img_ptr == tracked_obj_name) { // tracked object color
        (*components_illus_ptr)[2] = TRACKED_OBJECT_COLOR[0]; // blue
        (*components_illus_ptr)[1] = TRACKED_OBJECT_COLOR[1]; // green
        (*components_illus_ptr)[0] = TRACKED_OBJECT_COLOR[2]; // red
        // white contour for the tracked object
        // if next or previous value is not tracked object
        if ((value_idx > 0 // left
             && *(objects_names_img_ptr + 1) != tracked_obj_name)
            || (value_idx < n_values - 1 // right
                && *(objects_names_img_ptr - 1) != tracked_obj_name)
            || (value_idx > ncols // up
                && *(objects_names_img_ptr - ncols) != tracked_obj_name)
            || (value_idx < n_values - 1 - ncols // right
                && *(objects_names_img_ptr + ncols) != tracked_obj_name)) {
          cv::circle(components_illus,
                     cv::Point(value_idx % ncols,
                               value_idx / ncols),
                     2, cv::Scalar(255, 255, 255), -1);
        } // end if next or previous value is not tracked object
      } // end if (*objects_names_img_ptr == tracked_obj_name)
      else { // not tracked object -> random color acording to index
        color_utils::indexed_color255((*components_illus_ptr)[0],
            (*components_illus_ptr)[1],
            (*components_illus_ptr)[2],
            *objects_names_img_ptr);
      }
    } // end if (*objects_names_img_ptr != NO_OBJECT)
    ++objects_names_img_ptr;
    ++components_illus_ptr;
  } // end loop value_idx

  // scale img if needed
  if (scale == 1)
    return;
  if (resized_components_illus == NULL) {
    ROS_WARN("paint_object_image(): Cannot resize components_illus, "
             "the buffer is NULL");
    return;
  }
  // cf http://opencv.willowgarage.com/documentation/cpp/
  //          geometric_image_transformations.html#cv-resize
  cv::resize(components_illus, *resized_components_illus, cv::Size(),
             scale, scale, cv::INTER_NEAREST);
} // end paint_object_image();

} // end namespace fast_cluster_functions


#endif // FAST_CLUSTER_FUNCTIONS_H
