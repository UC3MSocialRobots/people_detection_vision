/*!
  \file        fast_cluster_selector.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/2

  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________

  A GUI to select a cluster of interest in the objects detected by
  the fast cluster detector.

\section Parameters
  - \b resize_scale
    [double] (default: 1)
    The scale of the interface.
    The original image is 320x240,
    the GUI size scales it by this parameter.

 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// AD
#include "vision_utils/timer.h"
// people_msgs
#include "people_detection_vision/fast_cluster_functions.h"

cv_bridge::CvImageConstPtr object_bridge_img;
bool bridge_img_ptr_reset = false;
cv::Mat3b _components_illus (1, 1);
cv::Mat _objects_img_copy;
int _obj_name= fast_cluster_functions::NO_OBJECT;
ros::Publisher _obj_coordinates_pub;
std_msgs::Int32 msg;

cv::Mat3b _components_illus_resized (1, 1);
double _resize_scale = 1;
const std::string WINDOW_NAME = fast_cluster_functions::OBJECTS_NAMES_IMAGE_TOPIC;

////////////////////////////////////////////////////////////////////////////////

void mouse_callback(int event, int x, int y, int, void*) {
  // only keep left button clicks
  if (event != CV_EVENT_LBUTTONDOWN)
    return;
  cv::Point comp_pos(x / _resize_scale, y / _resize_scale);
# if 0
  _obj_name = object_bridge_img->image.at<int>(comp_pos);

  if (_obj_name == fast_cluster_functions::NO_OBJECT) {
    ROS_WARN("The user clicked on a non object point. Stopping to track.");
  } else {
    ROS_WARN("Starting to track object %i.", _obj_name);
  }
#else // just send coordinates
  _obj_name = comp_pos.y * object_bridge_img->image.cols + comp_pos.x;
  ROS_WARN("Publishing %i (%i, %i).", _obj_name, comp_pos.x, comp_pos.y);
#endif
  msg.data = _obj_name;
  _obj_coordinates_pub.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////

//! create the image
void convert() {
  bridge_img_ptr_reset = false;

#if 0
  const cv::Mat1i* _objects_img_ptr =
      (const cv::Mat1i*) &(object_bridge_img->image);
  _objects_img_ptr->copyTo(_objects_img_copy);
  fast_cluster_functions::paint_object_image
      (_objects_img_copy, _components_illus, _obj_name,
       _resize_scale, &_components_illus_resized);
#else
  const cv::Mat3b* _objects_img_ptr =
      (const cv::Mat3b*) &(object_bridge_img->image);
  _objects_img_ptr->copyTo(_components_illus);
  cv::resize(_components_illus, _components_illus_resized, cv::Size(),
             _resize_scale, _resize_scale, cv::INTER_NEAREST);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO_THROTTLE(5, "fast_cluster_selector:image_callback()");
  // vision_utils::Timer timer;
  try {
    object_bridge_img = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // timer.printTime("image_callback();");
  bridge_img_ptr_reset = true;
} // end image_callback();

////////////////////////////////////////////////////////////////////////////////

//! set the new tracked object
void tracked_object_name_callback(const std_msgs::Int32ConstPtr& pt_msg) {
  _obj_name = pt_msg->data;
  ROS_INFO("fast_cluster_selector:Now tracking object %i", _obj_name);
} // end tracked_object_name_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_cluster_selector");
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  nh_private.param("resize_scale", _resize_scale, _resize_scale);

  // subscribe to image topic
  image_transport::ImageTransport transport(nh_public);
#if 0
  image_transport::Subscriber sub = transport.subscribe
      (fast_cluster_functions::OBJECTS_NAMES_IMAGE_TOPIC, 1, image_callback);
#else
  image_transport::Subscriber sub = transport.subscribe
      (fast_cluster_functions::COMPONENTS_ILLUS_TOPIC, 1, image_callback);
#endif
  // tracking publishers and subscribers
  ros::Subscriber obj_coordinates_subscriber = nh_public.subscribe
      (fast_cluster_functions::TRACKED_OBJECT_COORDINATES_TOPIC, 1,
       tracked_object_name_callback);

  // create publisher
  _obj_coordinates_pub = nh_public.advertise<std_msgs::Int32>
      (fast_cluster_functions::TRACKED_OBJECT_COORDINATES_TOPIC, 1);

  // create window
  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, mouse_callback);

  // spin!
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(15);
  while (ros::ok()) {
    if (bridge_img_ptr_reset)
      convert();
    if (_resize_scale == 1)
      cv::imshow(WINDOW_NAME, _components_illus);
    else
      cv::imshow(WINDOW_NAME, _components_illus_resized);
    // wait for key and kill if needed
    if (cv::waitKey(5) == 27)
      ros::shutdown();

    rate.sleep();
  }
  return 0;
}


