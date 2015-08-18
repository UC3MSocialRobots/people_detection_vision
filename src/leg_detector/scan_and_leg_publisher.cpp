/*!
  \file        scan_and_leg_publisher.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/9/12

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

Reads CSV file containing laser scans along with ground truth user positions.
Publishes the former in ROS LaserScans and the latter in PPL messages.

\section Parameters
  - \b "~csv_filename"
        [string] (default: "data.csv")
        Where to get the laser scans.

  - \b "~scan_unit2meters"
        [double] (default: 1.0)
        How to convert the units of the scans into meters, useful if
        for instance scans are in millimeters.

  - \b "~laser_topic"
        [string] (default: "base_scan")
        Where to publish the laser scans.

\section Subscriptions
    None.

\section Publications
  - \b {laser_topic}
        [sensor_msgs::LaserScan]
        The laser scans read from the file.

  - \b "~ppl"
        [people_msgs::PeoplePoseList]
        The poses of the found people.
 */
// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
// people_msgs
#include "people_utils/pplp_template.h"
// utils
#include <string/file_io.h>
#include <string/string_split.h>
#include <geom/foo_point.h>

#define DEG2RAD 0.01745329251994329577
#define RAD2DEG 57.2957795130823208768

class ScanAndLegPublisher : public PPLPublisherTemplate {
public:
  //! a minimalistic Point structure
  typedef geometry_utils::FooPoint2f Point2;

  //////////////////////////////////////////////////////////////////////////////

  static Point2 point_from_polar(const double & r, const double & theta) {
    Point2 ans;
    ans.x = r * cos(theta);
    ans.y = r * sin(theta);
    return ans;
  }

  //////////////////////////////////////////////////////////////////////////////

  ScanAndLegPublisher() : PPLPublisherTemplate("scan_and_leg_publisher_START", "scan_and_leg_publisher_STOP") {
    _curr_line = 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // params
    std::string csv_filename = "data.csv";
    std::string laser_topic = "scan";
    _scan.header.frame_id = "scan";
    _nh_private.param("csv_filename", csv_filename, csv_filename);
    _nh_private.param("laser_topic", laser_topic, laser_topic);
    _nh_private.param("angle_min", _angle_min, -135. * DEG2RAD);
    _nh_private.param("angle_max", _angle_max, 135. * DEG2RAD);
    _nh_private.param("scan_unit2meters", _scan_unit2meters, 1.);
    _nh_private.param("frame_id", _scan.header.frame_id, _scan.header.frame_id);

    // publishers
    _laser_pub = _nh_public.advertise<sensor_msgs::LaserScan>(laser_topic, 1);

    // parse file
    if (!StringUtils::retrieve_file_split(csv_filename, _lines, false, true)) {
      printf("Could not read CSV '%s'!\n", csv_filename.c_str());
    }

    printf("scan_and_leg_publisher:This executable loads data from '%s'"
           ", publishes laser scans on '%s', and PeoplePoseList results to '%s'.",
           csv_filename.c_str(),
           _laser_pub.getTopic().c_str(), get_ppl_topic().c_str());
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    _laser_pub.shutdown();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool publish_next_line() {
    if (_curr_line > _lines.size() - 3) { // EOF
      printf("Reached end of file!\n");
      return false;
    }
    // read scan
    StringUtils::StringSplit_<float>(_lines[_curr_line+1], ",", &_scan.ranges);
    if (fabs(_scan_unit2meters - 1) > 1E-3)
      //_scan.ranges *= _scan_unit2meters;
      std::transform(_scan.ranges.begin(), _scan.ranges.end(), _scan.ranges.begin(),
                     std::bind1st(std::multiplies<float>(),_scan_unit2meters));
    unsigned int nranges = _scan.ranges.size();
    if (nranges < 2) {
      printf("Incorrect ranges line '%s'!\n", _lines[_curr_line+1].c_str());
      return false;
    }
    _scan.header.stamp = ros::Time::now();
    _scan.angle_min = _angle_min;
    _scan.angle_max = _angle_max;
    _scan.angle_increment = (_angle_max - _angle_min) / nranges;

    // read user pos
    std::vector<float> users_positions;
    StringUtils::StringSplit_<float>(_lines[_curr_line+2], ",", &users_positions);
    unsigned int nusers = users_positions.size() / 3;
    // build new message
    _ppl.header = _scan.header;
    _ppl.method = "scan_and_leg_publisher";
    _ppl.poses.clear();
    _ppl.poses.reserve(nusers);
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      people_msgs::PeoplePose people_pose;
      people_pose.header = _ppl.header; // copy header
      people_pose.person_name = StringUtils::cast_to_string(user_idx);
      people_pose.confidence = 1;
      people_pose.std_dev = 0;
      // set pose
      people_pose.head_pose.orientation = tf::createQuaternionMsgFromYaw(0);
      people_pose.head_pose.position.x = users_positions[user_idx*3];
      people_pose.head_pose.position.y = users_positions[user_idx*3+1];
      people_pose.head_pose.position.z = 1.7;
      // add it
      _ppl.poses.push_back(people_pose);
    } // end loop user_idx
    // publish message

    // publish data
    _laser_pub.publish(_scan);
    publish_PPL(_ppl);
    _curr_line+=3;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! ROS message to share results
  people_msgs::PeoplePoseList _ppl;
  ros::Publisher _laser_pub;
  double _angle_min, _angle_max, _scan_unit2meters;
  std::vector<std::string> _lines;
  sensor_msgs::LaserScan _scan;
  unsigned int _curr_line;
}; // end class ScanAndLegPublisher

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_and_leg_publisher");
  ScanAndLegPublisher skill;
  skill.start();
  ros::Rate r(5);
  while(ros::ok() && skill.publish_next_line()) {
    ros::spinOnce();
    r.sleep();
  }
  skill.stop();
  return 0;
}
