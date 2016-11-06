/*!
  \file        artoolkit_pplp.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/9

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

A people detector and people_msgs::People publisher
based on ARToolkit tags.

\section Parameters
  - \b "~input_caminfo_topic"
        [string] (default: "rgb/camera_info")
        Where to get the input camera infos of the Kinect or the camera.
        One of these messages will be stored.
        When the skill is activated, it will be published on the cam info
        topic of the 'ar_pose' node (cf param "~artoolkit_caminfo_topic"
        which will activate it.

  - \b "~artoolkit_caminfo_topic"
        [string] (default: "ar/camera_info")
        The topic listened to by the 'ar_pose' node.

  - \b "~artoolkit_markers_topic"
        [string] (default: "ar_pose_marker")
        The output topic of the 'ar_pose' node.

\section Subscriptions
  - \b "{input_caminfo_topic}"
        [sensor_msgs/CameraInfo]
        Not a proper subscription, done with a ros::topic::waitForMessage.

  - \b "{artoolkit_markers_topic}"
        [ar_pose/ARMarkers]
        The output of the 'ar_pose' node.

\section Publications
  - \b "{artoolkit_caminfo_topic}"
        [sensor_msgs/CameraInfo]
        The camera info that activates the 'ar_pose' node.

  - \b "~ppl"
        [people_msgs::People]
        The detected users in the mask
 */

// ROS
#include <sensor_msgs/CameraInfo.h>
#include <ros/topic.h>
// AD
#include <people_msgs/People.h>
#include "vision_utils/pplp_template.h"
#include "vision_utils/ppl_attributes.h"
#include "vision_utils/mean.h"
#include "vision_utils/exec_system_get_output.h"
#include "people_detection_vision/artoolkit_utils.h"

class ARToolkitPPLP : public vision_utils::PPLPublisherTemplate {
public:
  typedef sensor_msgs::CameraInfo CamInfo;

  ARToolkitPPLP()
    : PPLPublisherTemplate("ARTOOLKIT_PPLP_START", "ARTOOLKIT_PPLP_STOP") {
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    _art_caminfo_topic = "ar/camera_info";
    _nh_private.param("artoolkit_caminfo_topic",
                      _art_caminfo_topic,
                      _art_caminfo_topic);
    _art_markers_topic = "ar_pose_marker";
    _nh_private.param("artoolkit_markers_topic",
                      _art_markers_topic,
                      _art_markers_topic);
    _marker_pattern_list = "";
    _nh_private.param("marker_pattern_list",
                      _marker_pattern_list,
                      _marker_pattern_list);
    std::string input_caminfo_topic = "rgb/camera_info";
    _nh_private.param("input_caminfo_topic",
                      input_caminfo_topic,
                      input_caminfo_topic);
    _nh_private.resolveName(input_caminfo_topic);
    printf("ARToolkitPPLP: trying to get camer info on '%s'\n",
           input_caminfo_topic.c_str());
    // wait for camera info
    CamInfo::ConstPtr caminfo_ptr = ros::topic::waitForMessage<CamInfo>
                                    (input_caminfo_topic, _nh_public, ros::Duration(15));
    // if not obtained, die!
    if (!caminfo_ptr) {
      ROS_FATAL("Could not get Kinect cam info on '%s', dying, argh!",
                input_caminfo_topic.c_str());
      ros::shutdown();
    }
    // keep it
    _caminfo = *caminfo_ptr;
    _caminfo_pub = _nh_public.advertise<CamInfo>(_art_caminfo_topic, 1, false); // no latch
    if (_marker_pattern_list.size() > 0) {
      _pattern_map.parse(_marker_pattern_list);
      ROS_INFO("Pattern map: loaded %i patterns: '%s'",
               _pattern_map.nkeys(), _pattern_map.to_string().c_str());
    }

    // publish camera info for artoolkit
    sleep(1);
    if (_caminfo_pub.getNumSubscribers() < 1) {
      ROS_WARN("Seems like the artoolkit node isnt listening to our camera info "
               "on '%s'", _caminfo_pub.getTopic().c_str());
    }
    _caminfo_pub.publish(_caminfo);

    // subscribe to markers
    _art_markers_sub = _nh_public.subscribe(_art_markers_topic, 1,
                                            &ARToolkitPPLP::markers_cb, this);

    ROS_INFO("ARToolkitPPLP: subscribing to '%s', publishing cam info on '%s', "
             "publishing PPL to '%s'",
             _art_markers_sub.getTopic().c_str(),
             _caminfo_pub.getTopic().c_str(),
             get_ppl_topic().c_str());

  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  void markers_cb(const ar_pose::ARMarkers::ConstPtr & msg) {
    unsigned int nmarkers = msg->markers.size();
    ROS_INFO_THROTTLE(1, "markers_cb(%i markers)",nmarkers);
    if (get_ppl_num_subscribers() == 0)
      return;
    // convert to PPL
    // prepare the message
    people_msgs::People ppl_msg;
    // ppl_msg.header = msg->header;
    // the header of the marker list is empty but not the header of each marker
    if (nmarkers >= 1)
      ppl_msg.header = msg->markers[0].header;
    else {
      ppl_msg.header.frame_id = _caminfo.header.frame_id;
      ppl_msg.header.stamp = ros::Time::now();
    }
    ppl_msg.people.reserve(nmarkers);

    // convert center of masses to 3D positions
    for (unsigned int marker_idx = 0; marker_idx < nmarkers; ++marker_idx) {
      ar_pose::ARMarker marker = msg->markers[marker_idx];
      // shape a Person
      people_msgs::Person pp;
      vision_utils::set_method(pp, "artoolkit");
      std::string name;
      if (_pattern_map.direct_lookup(marker.id, name)) // conversion id -> name
        pp.name = name;
      else // keep id
        pp.name = vision_utils::cast_to_string(marker.id);
      pp.position = marker.pose.pose.position;
      // http://en.wikipedia.org/wiki/Covariance_and_correlation
      // The covariance of a variable with itself (i.e. \sigma_{XX})
      // is called the variance and is more commonly denoted as \sigma_X^2,
      // the square of the standard deviation
      //double mean_cov = -1;//, stddev_cov;
      //mean_std_dev(marker.pose.covariance, mean_cov, stddev_cov);
      //vision_utils::mean<double>((double*) &(marker.pose.covariance), 36);
      //pp.std_dev = mean_cov;
      pp.reliability = 0.01 * marker.confidence; // 0..100 -> 0..1
      ppl_msg.people.push_back(pp);
    } // end loop marker_idx
    publish_PPL(ppl_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers()  {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    // unsubscribe to markers
    _art_markers_sub.shutdown();

    // kill the node
    std::string out = vision_utils::exec_system_get_output
                      ("rosnode kill ar_pose");
    // correct output:
    // killing /input_only/ar_pose
    // killed
    // wrong output, on std::cerr:
    // ERROR: Unknown node(s):
    //  * /input_only/ar_poseww
    //printf("out:'%s'\n", out.c_str());
    if (out.find("killed") == std::string::npos) {
      ROS_WARN("Seems like the 'ar_pose' node didn't die!");
    }
  } // end shutdown_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

private:
  CamInfo _caminfo;
  std::string _art_caminfo_topic;
  ros::Publisher _caminfo_pub;

  ros::Subscriber _art_markers_sub;
  std::string _art_markers_topic;

  std::string _marker_pattern_list;
  artoolkit_utils::Id2PatternName _pattern_map;
}; // end ARToolkitPPLP

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init (argc, argv, "ARToolkitPPLP");
  ARToolkitPPLP skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}
