/*!
  \file        fast_cluster_joy_control.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/4/12

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

 */


#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
// people_msgs
#include "people_detection_vision/fast_cluster_functions.h"

ros::Subscriber joy_sub_;
ros::Publisher _object_publisher;
ros::Publisher _reset_markers_publisher;

////////////////////////////////////////////////////////////////////////////////

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // ROS_INFO("joyCallback()");

  /* A super nice drawing for the code of the buttons on the
     Logitech RumblePad 2:
     ---------------
    |  [6]     [7]  |
    |  [4]     [5]  |
     ---------------
    |   |      (3)  |
    |  -+-   (0) (2)|
    |   |      (1)  |
    / /-----------\ \
   / /             \ \
    */

  if (joy->buttons[4] || joy->buttons[6]) {
    ROS_INFO_THROTTLE(1, "Emitting a cancel message");
    std_msgs::Int32 msg; msg.data = fast_cluster_functions::NO_OBJECT;
    _object_publisher.publish(msg);
  }
  else if (joy->buttons[0] || joy->buttons[1] || joy->buttons[2] || joy->buttons[3] ) {
    ROS_INFO_THROTTLE(1, "Clearing markers");
    _reset_markers_publisher.publish(std_msgs::Empty());
  }
  // buttons 5 and 7 are used for the followme laser

} // end joyCallback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_cluster_joy_control");
  // ROS_INFO("fast_cluster_joy_control main");
  ros::NodeHandle nh_public;

  // create publisher
  joy_sub_ = nh_public.subscribe<sensor_msgs::Joy>("joy", 2, joyCallback);
  _object_publisher = nh_public.advertise<std_msgs::Int32>
      (fast_cluster_functions::TRACKED_OBJECT_COORDINATES_TOPIC, 1);
  _reset_markers_publisher = nh_public.advertise<std_msgs::Empty>
      ("trail_markers_reset", 1);

  // spin!
  ros::spin();
  return 0;
}

