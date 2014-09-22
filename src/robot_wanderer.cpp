/*!
  \file         robot_wanderer.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2012/1

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

  This node will make the robot navigate within the free space given by the laser.

*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
// people_msgs
#include <src/ros_utils/marker_utils.h>
#include <src/ros_utils/odom_utils.h>
#include <src/ros_utils/laser_utils.h>
#include <src/geom/distances.h>
#include <src/geom/geometry_utils.h>
#include <src/time/timer.h>

/*

  */

//! the maximal velocities, m/s or rad/s
double _min_vel_lin = .1, _max_vel_lin = .3, _max_vel_ang = .5;
//! the current velocities, m/s or rad/s
float _vel_lin = 0.1, _vel_ang = 0.1;
//! the distance we want to keep away from obstacles, meters
double min_obstacle_distance = .4;

//! the forecast time in seconds
static const float TIME_PRED = 5;
//! the time step simulation
static const float DT = 0.2;

//! a maximum time before we choose some other speeds
static const float speed_timeout = 5;

static const int MAX_TRIES = 100000;

// TYPEDEFS
typedef geometry_utils::FooPoint2f Pt2;
// DATA
//! the publishers
ros::Publisher vel_pub, _marker_pub;
//! the vizu marker
visualization_msgs::Marker _marker;
//! the laser data in (x, y) frame
std::vector<Pt2> laser_xy;
//! timer since last computed speed
Timer speed_timer;

////////////////////////////////////////////////////////////////////////////////

/*!
  send a 0 speed
*/
void stop_robot() {
  ROS_INFO("stop_robot_and_exit()");
  geometry_msgs::Twist out;
  vel_pub.publish(out);
}

////////////////////////////////////////////////////////////////////////////////

/*!
  send a 0 speed to the robot and exit program
 \param param
    the exit code returned
*/
void stop_robot_and_exit(int param = 0) {
  ROS_INFO("stop_robot_and_exit()");
  stop_robot_and_exit();
  ros::shutdown();
  exit(param);
}

////////////////////////////////////////////////////////////////////////////////

//! return true if the trajectory wont collide with the laser in the time TIME_PRED
bool check_trajectory_valid(const float & vel_lin, const float & vel_ang,
                            const std::vector<Pt2> & laser_xy) {
  // determine the coming trajectory
  std::vector<Pt2> traj_xy;
  odom_utils::make_trajectory(vel_lin, vel_ang, traj_xy, TIME_PRED, DT, 0, 0, 0);
  // find if there might be a collision
  if (geometry_utils::two_vectors_closer_than_threshold(traj_xy, laser_xy, min_obstacle_distance))
    return false;
  else
    return true;
}

////////////////////////////////////////////////////////////////////////////////

void laser_callback(const sensor_msgs::LaserScanConstPtr & laser_in) {
  ROS_INFO_THROTTLE(1, "laser_callback()");
  //ROS_INFO("laser_callback()");

  // convert laser data to xy
  laser_utils::convert_sensor_data_to_xy(*laser_in, laser_xy);
  //  marker_utils::list_points2_as_primitives(marker, laser_xy, "laser_xy", 0.1, 0.01, 0, 1, 0);
  //  marker_pub.publish(marker);

  bool want_recompute_speed = false;
  // recompute speeds if the last ones are too old
  if (!want_recompute_speed && speed_timer.getTimeSeconds() > speed_timeout) {
    ROS_INFO("speed_timeout");
    want_recompute_speed = true;
  }
  // check if there will be a collision soon with the elected speeds
  if (!want_recompute_speed &&
      check_trajectory_valid(_vel_lin, _vel_ang, laser_xy) == false) {
    ROS_INFO("coming_collision !");
    want_recompute_speed = true;
  } // end

  if (want_recompute_speed) {
    bool new_speeds_found = false;
    speed_timer.reset();
    int nb_tries = 0;
    // choose a random speed and check it enables at least 1 second of movement
    while (!new_speeds_found) {
      // authorize on place rotations only after having tried many times
      float curr_min_vel_lin = (nb_tries < 100 ? _min_vel_lin : 0);
      if (1.f * nb_tries / MAX_TRIES > .9 )
        curr_min_vel_lin = -.1; // authorize backward moves in extreme cases
      _vel_lin = curr_min_vel_lin + drand48() * (_max_vel_lin - curr_min_vel_lin);
      _vel_ang = drand48() * 2 * _max_vel_ang - _max_vel_ang;
      new_speeds_found = check_trajectory_valid(_vel_lin, _vel_ang, laser_xy);
      if (nb_tries++ > MAX_TRIES)
        break;
    } // end while (!new_speeds_found)

    if (!new_speeds_found) {
      ROS_ERROR("The robot is stuck! Stopping motion and exiting.");
      stop_robot();
      return;
    } // end not new_speeds_found

    ROS_INFO("Found a suitable couple of speeds in %g ms and %i tries: "
             "_vel_lin:%g, _vel_ang:%g",
             speed_timer.getTimeMilliseconds(), nb_tries, _vel_lin, _vel_ang);
    speed_timer.reset();
  } // end if (want_recompute_speed)

  std::vector<Pt2> traj_xy;
  odom_utils::make_trajectory(_vel_lin, _vel_ang, traj_xy, TIME_PRED, DT, 0, 0, 0);
  marker_utils::list_points2_as_primitives(_marker, traj_xy, "traj_xy", 0.1, 0.03, 1, 0, 0);
  _marker_pub.publish(_marker);

  // publish the computed speed
  geometry_msgs::Twist out;
  out.angular.z = _vel_ang;
  out.linear.x = _vel_lin;
  ROS_INFO_THROTTLE(1, "Publishing _vel_lin:%g, _vel_ang:%g", _vel_lin, _vel_ang);
  vel_pub.publish(out);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_wanderer");
  ros::NodeHandle nh_public, nh_private("~");

  // stop the robot if signal received
  signal(SIGINT, stop_robot_and_exit);
  signal(SIGTERM, stop_robot_and_exit);
  signal(SIGKILL, stop_robot_and_exit);

  // get params
  std::string laser_topic, cmd_vel_topic;
  nh_private.param<std::string>("laser_topic", laser_topic, "scan_filtered");
  nh_private.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
  nh_private.param<double>("min_vel_lin", _min_vel_lin, _min_vel_lin);
  nh_private.param<double>("max_vel_lin", _max_vel_lin, _max_vel_lin);
  nh_private.param<double>("max_vel_ang", _max_vel_ang, _max_vel_ang);
  nh_private.param<double>("min_obstacle_distance", min_obstacle_distance, min_obstacle_distance);
  ROS_INFO("Laser scans from '%s', emitting speeds to '%s'",
           laser_topic.c_str(), cmd_vel_topic.c_str());

  ros::Subscriber sub = nh_public.subscribe(laser_topic, 1, laser_callback);
  vel_pub = nh_public.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  _marker_pub = nh_public.advertise<visualization_msgs::Marker>
      ("robot_wanderer_traj", 1);

  ros::spin();
  stop_robot_and_exit();
  return 0;
}
