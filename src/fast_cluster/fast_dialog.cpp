/*!
  \file        fast_dialog.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/4

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

\todo Description of the file

\section Parameters
  - \b "followme_laser_node_name"
        [string] (default: "followme_laser_node")
        The name of the laser followme node.

  - \b "fast_cluster_detector_node_name"
        [string] (default: "fast_cluster_detector_node")
        The name of the fast cluster detector node.

\section Subscriptions
  - \b "{followme_laser_node_name}/status"
        [followme_laser/TrackingStatus]
        The status of the laser followme node.

  - \b "{fast_cluster_detector_node_name}/status"
        [followme_laser/TrackingStatus]
        The status of the fast cluster detector node.

  - \b "{followme_laser_node_name}/moving_goal"
        [geometry_msgs::PoseStamped]
        The pose of the object being tracked by the laser followme node.

  - \b fast_cluster_functions::TRACKED_OBJECT_POSE_TOPIC
        [geometry_msgs::PoseStamped]
        The pose of the object being tracked by the fast cluster detector node.

\section Publications
  - \b "~moving_goal"
        [geometry_msgs::PoseStamped]
        The final pose of the object being tracked.

  - \b "{followme_laser_node_name}/set_tracking_seed"
        [geometry_msgs::PoseStamped]
        Sets the pose of the object being tracked by the followme_laser_node_name node.

  - \b "{fast_cluster_detector_node_name}/set_tracking_seed"
        [geometry_msgs::PoseStamped]
        Sets the pose of the object being tracked by the fast_cluster node.

 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// fast_cluster_functions
#include "fast_cluster_functions.h"

////////////////////////////////////////////////////////////////////////////////

ros::Subscriber _followme_laser_status_sub;
TrackingStatus _followme_laser_last_status;
ros::ServiceClient _followme_laser_set_tracking_seed_client;

ros::Subscriber _fast_cluster_detector_status_sub;
TrackingStatus _fast_cluster_detector_last_status;
ros::ServiceClient _fast_cluster_detector_set_tracking_seed_client;

ros::Publisher _moving_goal_pub;

////////////////////////////////////////////////////////////////////////////////

void followme_laser_status_cb
(const TrackingStatusConstPtr & msg)
{ _followme_laser_last_status = *msg; }

////////////////////////////////////////////////////////////////////////////////

void fast_cluster_detector_status_cb
(const TrackingStatusConstPtr & msg)
{  _fast_cluster_detector_last_status = *msg; }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_dialog");
  ros::NodeHandle nh_public, nh_private("~");

  // get params
  std::string followme_laser_node_name = "followme_laser_node",
      fast_cluster_detector_node_name = "fast_cluster_detector_node";
  nh_private.param("followme_laser_node_name",
                   followme_laser_node_name, followme_laser_node_name);
  nh_private.param("fast_cluster_detector_node_name",
                   fast_cluster_detector_node_name, fast_cluster_detector_node_name);

  // subscribers
  _followme_laser_status_sub = nh_public.subscribe
      (followme_laser_node_name + std::string("/status"), 1,
       followme_laser_status_cb);
  _fast_cluster_detector_status_sub = nh_public.subscribe
      (fast_cluster_detector_node_name + std::string("/status"), 1,
       fast_cluster_detector_status_cb);

  // publishers
  _moving_goal_pub = nh_private.advertise<geometry_msgs::PoseStamped>
      ("moving_goal", 1);
  _followme_laser_set_tracking_seed_client= nh_public.serviceClient
      <followme_laser::SetTrackingSeed>
      (followme_laser_node_name + std::string("/set_tracking_seed"));
  _fast_cluster_detector_set_tracking_seed_client= nh_public.serviceClient
      <followme_laser::SetTrackingSeed>
      (fast_cluster_detector_node_name + std::string("/set_tracking_seed"));

  ROS_WARN_STREAM
      ("fast_dialog: listening to:\n"
       << " * _followme_laser: "
       << _followme_laser_status_sub.getTopic() << " , "
       << " * _fast_cluster_detector: "
       << _fast_cluster_detector_status_sub.getTopic() << " , "
       << "publishing goal to " << _moving_goal_pub.getTopic() << "\n"
       << "and corrections to "
       << _followme_laser_set_tracking_seed_client.getService()
       << " and"
       << _fast_cluster_detector_set_tracking_seed_client.getService());

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(10);
  while(ros::ok()) {
    //    ROS_WARN_THROTTLE
    //        (1, "_followme_laser: status=%i,  _fast_cluster_detector: status=%i",
    //         _followme_laser_last_status.status,
    //         _fast_cluster_detector_last_status.status);

    // take a decision here

    if (_followme_laser_last_status.status == TrackingStatus::NO_TARGET
        && _fast_cluster_detector_last_status.status == TrackingStatus::NO_TARGET) {
      ROS_INFO_THROTTLE(1, "Both detectors are idle (no target).");
    }

    if (_followme_laser_last_status.status == TrackingStatus::TARGET_TRACKING_OK
        && _fast_cluster_detector_last_status.status == TrackingStatus::TARGET_TRACKING_OK) {
      ROS_INFO_THROTTLE(1, "Both detectors are up and running.");
    }

    if (_followme_laser_last_status.status == TrackingStatus::TARGET_TRACKING_LOST
        && _fast_cluster_detector_last_status.status == TrackingStatus::TARGET_TRACKING_LOST) {
      ROS_WARN_THROTTLE(1, "Ow ow, both detectors are lost!");
    }

    bool goal_emitted = false;
#if 1
    // first try fast cluster
    if (!goal_emitted && _fast_cluster_detector_last_status.status ==
        TrackingStatus::TARGET_TRACKING_OK)
    {
      ROS_INFO_THROTTLE(1, "_fast_cluster_detector: tracking OK, "
                        "setting new seed to _followme_laser");
      // set the seed for the laser
      geometry_msgs::PoseStamped::ConstPtr fast_cluster_detector_last_moving_goal =
          ros::topic::waitForMessage<geometry_msgs::PoseStamped>
          (fast_cluster_functions::TRACKED_OBJECT_POSE_TOPIC, nh_public,
           ros::Duration(.5));
      if (fast_cluster_detector_last_moving_goal) {
        // set new seed for followme_laser
        if (_followme_laser_last_status.status == TrackingStatus::NO_TARGET) {
          ROS_INFO_THROTTLE (1, "Feeding _followme_laser");
          followme_laser::SetTrackingSeed srv;
          srv.request.pose_in = *fast_cluster_detector_last_moving_goal;
          bool srv_success = _followme_laser_set_tracking_seed_client.call(srv);
          if (!srv_success)
            ROS_WARN_THROTTLE
                (1, "Call of service %s failed.",
                 _followme_laser_set_tracking_seed_client.getService().c_str());
        }
        // publish the goal of the _fast_cluster_detector
        _moving_goal_pub.publish(fast_cluster_detector_last_moving_goal);
        goal_emitted = true;
      }
      else {
        ROS_WARN_THROTTLE(1, "Failed to get fast_cluster_detector_last_moving_goal");
      }
    } // end if (!goal_emitted && fast_cluster_detector status == TARGET_TRACKING_OK)
#endif

    if (!goal_emitted && _followme_laser_last_status.status ==
        TrackingStatus::TARGET_TRACKING_OK) {
      ROS_INFO_THROTTLE
          (1, "_fast_cluster_detector has lost the tracked object"
           " but _followme_laser tracking is OK.");
      // set new seed for followme_laser

      // publish the goal of the _followme_laser
      geometry_msgs::PoseStamped::ConstPtr followme_laser_last_moving_goal =
          ros::topic::waitForMessage<geometry_msgs::PoseStamped>
          (followme_laser_node_name + std::string("/moving_goal"), nh_public,
           ros::Duration(.5));
      if (followme_laser_last_moving_goal) {
        // set new seed for fast_cluster_detector
        if (_fast_cluster_detector_last_status.status == TrackingStatus::NO_TARGET) {
          ROS_INFO_THROTTLE (1, "Feeding _fast_cluster_detector");
          followme_laser::SetTrackingSeed srv;
          // try with different height increments, as the laser is pretty low
          for (double height_incr = 0; height_incr <= .5; height_incr +=.1) {
            srv.request.pose_in = *followme_laser_last_moving_goal;
            srv.request.pose_in.pose.position.z += height_incr;
            bool srv_success = _fast_cluster_detector_set_tracking_seed_client.call(srv);
            if (srv_success) {
              ROS_INFO_THROTTLE
                  (1, "Call of service %s with height_incr %g success.",
                   _fast_cluster_detector_set_tracking_seed_client.getService().c_str(),
                   height_incr);
              break;
            }
            ROS_WARN("Call of service %s with height_incr %g failed.",
                     _fast_cluster_detector_set_tracking_seed_client.getService().c_str(),
                     height_incr);
          } // end loop height_incr
        } // end if (_fast_cluster_detector status == NO_TARGET)

        // publish the goal of followme_laser
        _moving_goal_pub.publish(followme_laser_last_moving_goal);
        goal_emitted = true;
      }
      else {
        ROS_WARN_THROTTLE(1, "Failed to get followme_laser_last_moving_goal");
      }
    } // end if (!goal_emitted && followme_laser status == TARGET_TRACKING_OK)

    rate.sleep();
  } // end while(ros::ok())
  return 0;
} // end main()
