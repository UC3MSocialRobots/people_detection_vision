/*!
  \file        robot_info_grabber.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/1
________________________________________________________________________________

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
________________________________________________________________________________

A node for saving data from a mobile robot,
namely rgb and depth images stream, and odometry.
All fields are optional, so it can also be used for
a robot with a monocular camera,
or another without odometry.

\section Parameters
  - \b "~rgb_topic", "~depth_topic"
        [string] (default: "")
        Where to get the RGB or depth stream.
        Leave empty for not saving this data.

  - \b "~odom_topic"
        [string] (default: "")
        Where to get the odometry messages.
        Leave empty for not saving this data.

  - \b "~output_folder"
        [string] (default: "")
        Where the captured data is saved.
        If empty, the default folder "/tmp/robot_info_grabber_" + timestamp
        is used.

  - \b "~save_images_yaml"
        [bool] (default: true)
        Save depth and RGB images (if subscribed) in yaml fileformat.
        The generated file weights about 10 MB for a pair of RGB+depth frames
        in VGA resolution.

  - \b "~save_images_png"
        [bool] (default: true)
        Save depth and RGB images (if subscribed) in PNG fileformat.
        The generated file weights about 500 kB for a pair of RGB+depth frames
        in VGA resolution.

\section Subscriptions
  - \b  {rgb_topic}, {depth_topic}
        [sensor_msgs/Image]
        The input images.
        If both RGB and depth topics are not empty,
        they will be subscribed simultaneously
        (\see message_filters::sync_policies::ApproximateTime )

  - \b  {odom_topic}
        [nav_msgs/Odometry]
        The input odometry

\section Publications
  None
 */

// boost
#include "boost/filesystem.hpp" // for creating folders
#include "vision_utils/cv_conversion_float_uchar.h"

// utils
#include "vision_utils/io.h"
#include <ros_utils/pt_utils.h>
#include <string/StringUtils.h>
// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// images
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// odometry
#include <nav_msgs/Odometry.h>

// params
bool _subscribe_odom, _subscribe_depth, _subscribe_rgb;
bool _save_images_png = true;
bool _save_images_yaml = true;
std::string _output_folder;
ros::Time _last_time;
ros::Time _begin_time;
int _frames_grabbed = 0;
// rgb stuff
cv_bridge::CvImageConstPtr _rgb_ptr;
//depth stuff
cv_bridge::CvImageConstPtr _depth_ptr;
cv::Mat _depth_img_as_uchar, _src_float_clean_buffer;
image_utils::ScaleFactorType _alpha, _beta;
// odom
typedef nav_msgs::Odometry OdomMsg;
OdomMsg _last_odom;

inline void freq_reset() {
  ++_frames_grabbed;
  _last_time = ros::Time::now();
}
inline double freq_get() {
  ros::Time _now = ros::Time::now();
  return 1.f / (_now - _last_time).toSec();
}

inline double time() {
  return (ros::Time::now() - _begin_time).toSec();
}

////////////////////////////////////////////////////////////////////////////////

bool save_rgb_as_png(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const std::string & filename_prefix) {
  image_utils::write_rgb_and_depth_image_as_uchar_to_image_file
      (filename_prefix, &_rgb_ptr->image, NULL, NULL, NULL,
       image_utils::FILE_BMP, false);
  return true;
} // end save_rgb_as_png()

////////////////////////////////////////////////////////////////////////////////

bool save_depth_as_png(const sensor_msgs::ImageConstPtr& depth_msg,
                       const std::string & filename_prefix) {
  //image_utils::write_rgb_and_depth_image_to_image_file(filename_prefix, NULL, &depth_ptr->image);
  image_utils::convert_float_to_uchar(_depth_ptr->image,
                                      _depth_img_as_uchar, _src_float_clean_buffer,
                                      _alpha, _beta);
  image_utils::write_rgb_and_depth_image_as_uchar_to_image_file
      (filename_prefix, &_rgb_ptr->image, &_depth_img_as_uchar, &_alpha, &_beta,
       image_utils::FILE_BMP, false);
  return true;
} // end save_depth_as_png()

////////////////////////////////////////////////////////////////////////////////

bool save_last_odom_as_yaml(const std::string & filename_prefix) {
  // generate a string representation
  std::ostringstream odom_yaml;
  odom_yaml << "odom:\n";
  odom_yaml << "\tx:"<< _last_odom.pose.pose.position.x << "\n";
  odom_yaml << "\ty:"<< _last_odom.pose.pose.position.y << "\n";
  double roll, pitch, yaw;
  pt_utils::rpy_from_quaternion(_last_odom.pose.pose.orientation,
                                roll, pitch, yaw);
  odom_yaml << "\tyaw:"<< yaw << "\n";
  odom_yaml << "\torientation_quaternion:\n";
  odom_yaml << "\t\tx:"<< _last_odom.pose.pose.orientation.x << "\n";
  odom_yaml << "\t\ty:"<< _last_odom.pose.pose.orientation.y << "\n";
  odom_yaml << "\t\tz:"<< _last_odom.pose.pose.orientation.z << "\n";
  odom_yaml << "\t\tw:"<< _last_odom.pose.pose.orientation.w << "\n";

  odom_yaml << "speed:\n";
  odom_yaml << "\tlin:"<< _last_odom.twist.twist.linear.x << "\n";
  odom_yaml << "\tang:"<< _last_odom.twist.twist.angular.z << "\n";

  // save to file
  std::ostringstream odom_yaml_filename;
  odom_yaml_filename << filename_prefix << "_odom.yaml";
  std::ofstream myfile(odom_yaml_filename.str().c_str());
  if (myfile.is_open()) { // check if success
    myfile << odom_yaml.str();
    myfile.close();
  }
  return true;
} // end save_last_odom_as_yaml()

////////////////////////////////////////////////////////////////////////////////

void rgb_depth_cb(const sensor_msgs::ImageConstPtr& rgb_msg,
                  const sensor_msgs::ImageConstPtr& depth_msg) {
  std::string filename_prefix = _output_folder + std::string("/") + StringUtils::timestamp();
  ROS_INFO_THROTTLE(1, " %.2f Hz (%g s, %i frames), rgb_depth_cb('%s')",
                    freq_get(), time(), _frames_grabbed, filename_prefix.c_str());
  freq_reset();

  try { // decompress RGB and depth
    _rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    _depth_ptr = cv_bridge::toCvShare(depth_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (_save_images_png) {
    save_rgb_as_png(rgb_msg, filename_prefix);
    save_depth_as_png(depth_msg, filename_prefix);
  }
  if (_save_images_yaml)
    image_utils::write_rgb_and_depth_to_yaml_file
        (filename_prefix, _rgb_ptr->image, _depth_ptr->image, false);
  if (_subscribe_odom) // save odometry if wanted
    save_last_odom_as_yaml(filename_prefix);
} // end rgb_depth_cb();

////////////////////////////////////////////////////////////////////////////////

void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg) {
  std::string filename_prefix = _output_folder + std::string("/") + StringUtils::timestamp();
  ROS_INFO_THROTTLE(1, " %.2f Hz (%i frames), depth_cb('%s')",
                    freq_get(), _frames_grabbed, filename_prefix.c_str());
  freq_reset();

  try { // decompress RGB and depth
    _depth_ptr = cv_bridge::toCvShare(depth_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (_save_images_png)
    save_depth_as_png(depth_msg, filename_prefix);
  if (_save_images_yaml)
    image_utils::write_rgb_and_depth_to_yaml_file(filename_prefix, cv::Mat(), _depth_ptr->image);
  if (_subscribe_odom) // save odometry if wanted
    save_last_odom_as_yaml(filename_prefix);
} // end rgb_depth_odom_cb();

////////////////////////////////////////////////////////////////////////////////

void rgb_cb(const sensor_msgs::ImageConstPtr& rgb_msg) {
  std::string filename_prefix = _output_folder + std::string("/") + StringUtils::timestamp();
  ROS_INFO_THROTTLE(1, " %.2f Hz (%i frames), rgb_cb('%s')",
                    freq_get(), _frames_grabbed, filename_prefix.c_str());
  freq_reset();

  try { // decompress RGB and rgb
    _rgb_ptr = cv_bridge::toCvShare(rgb_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (_save_images_png)
    save_rgb_as_png(rgb_msg, filename_prefix);
  if (_save_images_yaml)
    image_utils::write_rgb_and_depth_to_yaml_file
        (filename_prefix, _rgb_ptr->image,cv::Mat(), false);
  if (_subscribe_odom) // save odometry if wanted
    save_last_odom_as_yaml(filename_prefix);
} // end rgb_rgb_odom_cb();

////////////////////////////////////////////////////////////////////////////////

void odom_cb(const OdomMsg::ConstPtr & odom_msg) {
  ROS_INFO_THROTTLE(10, "odom_cb()");
  // keep a copy
  _last_odom = *odom_msg;
  if (!_subscribe_depth && !_subscribe_rgb) { // save odometry if wanted
    std::string filename_prefix = _output_folder + std::string("/") + StringUtils::timestamp();
    save_last_odom_as_yaml(filename_prefix);
  }
} // end odom_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_info_grabber");
  ros::NodeHandle nh_public, nh_private("~");
  std::string rgb_topic = "";
  nh_private.param("rgb_topic", rgb_topic, rgb_topic);
  std::string depth_topic = "";
  nh_private.param("depth_topic", depth_topic, depth_topic);
  std::string odom_topic = "";
  nh_private.param("odom_topic", odom_topic, odom_topic);
  _output_folder = std::string("/tmp/robot_info_grabber_") + StringUtils::timestamp();
  nh_private.param("output_folder", _output_folder, _output_folder);
  nh_private.param("save_images_png", _save_images_png, _save_images_png);
  nh_private.param("save_images_yaml", _save_images_yaml, _save_images_yaml);

  _subscribe_rgb = (rgb_topic.size() > 0);
  _subscribe_depth = (depth_topic.size() > 0);
  _subscribe_odom = (odom_topic.size() > 0);

  ROS_INFO("robot_info_grabber:saving rgb:%i, depth:%i (PNG:%i, YAML:%i), odom:%i "
           "in output_folder:'%s'",
           _subscribe_rgb, _subscribe_depth, _save_images_png, _save_images_yaml,
           _subscribe_odom,
           _output_folder.c_str());
  //  std::ostringstream command;
  //  command << "mkdir --parents " << _output_folder;
  //  int retval = system_utils::exec_system(command.str());
  //  if (retval != 0) {
  boost::filesystem::path dir(_output_folder);
  if(!boost::filesystem::create_directory(dir)) {
    ROS_ERROR("Could not create folder '%s'", _output_folder.c_str());
    exit(-1);
  }

  ros::Subscriber rgb_sub, depth_sub, odom_sub;
  message_filters::Subscriber<sensor_msgs::Image> rgb_filter_sub, depth_filter_sub;
  message_filters::Subscriber<OdomMsg> odom_filter_sub;
  typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy>* sync = NULL;

  if (_subscribe_depth && _subscribe_rgb) {
    rgb_filter_sub.subscribe(nh_public, rgb_topic, 1);
    depth_filter_sub.subscribe(nh_public, depth_topic, 1);
    sync = new message_filters::Synchronizer<MySyncPolicy>
        (MySyncPolicy(5), rgb_filter_sub, depth_filter_sub);
    sync->registerCallback(boost::bind(&rgb_depth_cb, _1, _2));
  } // end if (subscribe_depth && subscribe_rgb)
  else if (_subscribe_rgb) // rgb, no depth
    rgb_sub = nh_public.subscribe(rgb_topic, 1, &rgb_cb);

  else if (_subscribe_depth) // depth, no rgb
    depth_sub = nh_public.subscribe(depth_topic, 1, &depth_cb);

  if (_subscribe_odom) {
    odom_sub = nh_public.subscribe(odom_topic, 1, &odom_cb);
    // we need an odometry message to start
    OdomMsg::ConstPtr msg =
        ros::topic::waitForMessage<OdomMsg>(odom_topic, nh_public, ros::Duration(1));
    if (!msg) {
      ROS_ERROR("Could not get an odometry message on '%s'", odom_topic.c_str());
      exit(-1);
    }
    odom_cb(msg);
  }

  ROS_INFO("Spinning...");
  _begin_time = ros::Time::now();
  freq_reset(); // init timer
  ros::spin();

  // clean
  if (_subscribe_depth && _subscribe_rgb)
    delete sync;
}
