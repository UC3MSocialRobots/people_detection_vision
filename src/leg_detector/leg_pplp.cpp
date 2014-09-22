/*!
  \file        leg_pplp.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/9/20

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

An implementation of:
N. Bellotto and H. Hu,
"Multisensor-based human detection and tracking for mobile service robots"
Systems, Man, and Cybernetics, Part B:  …, 2009.

\section Parameters
  - \b "edge_distance_threshold"
        [double] (default: .5)
        Description of the parameter.

  - \b "laser_src_topic"
        [string] (default: "base_scan")
        Where to get the laser scans.

  - \b "local_minimization_window_size"
        [int] (default: 3)
        The window of the local minimization filter
        aplied to the laser scan as preprocessing.

\section Subscriptions
  - \b {laser_src_topic}
        [sensor_msgs::LaserScan]
        The laser scans whee we will find the leg patterns

\section Publications
  - \b "~ppl"
        [people_msgs::PeoplePoseList]
        The poses of the found people.

  - \b "~marker"
        [visualization_msgs::Marker]
        The found people legs in a nice marker.


 */

#define PUBLISH_MARKER

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// people_msgs
#include <src/templates/pplp_template.h>
// utils
#include <src/ros_utils/pt_utils.h>
#include <src/data_filters/local_minimization_operator.h>
#include <src/geom/geometry_utils.h>
#include <src/geom/foo_point.h>
#include <src/string/string_casts.h>
#include <src/time/timer.h>
#ifdef PUBLISH_MARKER
#include "src/ros_utils/marker_utils.h"
#endif // PUBLISH_MARKER

class LegPPLP : public PPLPublisherTemplate {
public:
  //! a minimalistic Point structure
  typedef geometry_utils::FooPoint2f Point2;
  static const double ADJACENT_EDGES_DISTANCE_THRESH = .3 * .3; // m^2
  static const double ADJACENT_EDGES_ANGLE_THRESH = .2; // radians
  // distance parameters
  //! called "a" in the paper (adviced as 10 < a < 20 in part. IV)
  static const double LEG_PATTERN_LA_LEG_WIDTH = .15; // meters
  //! called "b" in the paper (adviced as b < 40 in part. IV)
  static const double LEG_PATTERN_LA_LEG_GAP = .3; // meters
  //! called "c" in the paper (adviced as 10 < c < 40 in part. IV)
  static const double LEG_PATTERN_SL_LEG_WIDTH = .35; // meters
  // max pattern distances
  static const double LEG_PATTERN_LA_MAX_DIST = .1;
  static const double LEG_PATTERN_FS_MAX_DIST = .2;
  static const double LEG_PATTERN_SL_MAX_DIST = .1;

  //////////////////////////////////////////////////////////////////////////////

  //! a minimalistic segment
  class Edge {
  public:
    Point2 p0, p1;
    // type of edge
    enum EDGE_TYPE { LEFT_EDGE, RIGHT_EDGE };
    EDGE_TYPE type;
  };

  //////////////////////////////////////////////////////////////////////////////

  static Point2 point_from_polar(const double & r, const double & theta) {
    Point2 ans;
    ans.x = r * cos(theta);
    ans.y = r * sin(theta);
    return ans;
  }

  //////////////////////////////////////////////////////////////////////////////

  static inline bool are_edges_adjacent(const Edge & e1, const Edge & e2) {
    // same type (left or right)
    if  (e1.type != e2.type)
      return false;
    // close matching ends
    if (geometry_utils::distance_points(e1.p1, e2.p0)
        > ADJACENT_EDGES_DISTANCE_THRESH)
      return false;
    // between 0 and PI
    if (fabs(M_PI -
             geometry_utils::absolute_angle_between_three_points
             (e1.p0, e1.p1, e2.p1)) > ADJACENT_EDGES_ANGLE_THRESH)
      return false;
    return true;
  } // end are_edges_adjacent()

  //////////////////////////////////////////////////////////////////////////////

  /*!
connect edges that are adjacent one to the other,
      and almost aligned
 \param in
    the concerned edges will be merged (the vector is changed)
*/
  static void connected_adjacent_edges(std::vector<Edge> & in) {
    for (unsigned int edge_idx = 0; edge_idx < in.size(); ++edge_idx) {
      for (unsigned int neigh_idx = edge_idx + 1;
           neigh_idx <= edge_idx + 3;
           ++neigh_idx) {
        // bound check
        if (neigh_idx >= in.size())
          break;
        // fuse if adjacent
        if (are_edges_adjacent(in[edge_idx], in[neigh_idx])) {
          // printf("Connecting %i and %i\n", edge_idx, neigh_idx);
          // merge both into in[edge_idx]
          in[edge_idx].p1 = in[neigh_idx].p1;
          // remove in[neigh_idx]
          in.erase(in.begin() + neigh_idx);
          // reset neightbour check
          neigh_idx = edge_idx;
        }
      } // end loop neigh_incr
    } // end loop edge_idx
  } // end connected_adjacent_edges();

  //////////////////////////////////////////////////////////////////////////////

  LegPPLP() : PPLPublisherTemplate("LEG_PPLP_START", "LEG_PPLP_STOP") {}

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // params
    _nh_private.param("laser_src_topic", laser_src_topic, std::string("base_scan"));
    _nh_private.param("edge_distance_threshold", edge_distance_threshold, .2);
    _nh_private.param("local_minimization_window_size", local_minimization_window_size, 3);

    // prepare patterns
    template_LA_pattern.clear();
    template_LA_pattern.push_back(Point2(0, 0));
    template_LA_pattern.push_back(Point2(LEG_PATTERN_LA_LEG_WIDTH, 0));
    template_LA_pattern.push_back(Point2(LEG_PATTERN_LA_LEG_GAP, 0));
    template_LA_pattern.push_back(Point2(LEG_PATTERN_LA_LEG_WIDTH, 0));
    candidate_LA_pattern.resize(4);

    template_FS_LLR_pattern.clear();
    template_FS_LLR_pattern.push_back(Point2(0, 0));
    template_FS_LLR_pattern.push_back(Point2(LEG_PATTERN_LA_LEG_WIDTH, -LEG_PATTERN_LA_LEG_WIDTH));
    template_FS_LLR_pattern.push_back(Point2(2 * LEG_PATTERN_LA_LEG_WIDTH, -LEG_PATTERN_LA_LEG_WIDTH));
    template_FS_LRR_pattern.push_back(Point2(0, 0));
    template_FS_LRR_pattern.push_back(Point2(LEG_PATTERN_LA_LEG_WIDTH, +LEG_PATTERN_LA_LEG_WIDTH));
    template_FS_LRR_pattern.push_back(Point2(2 * LEG_PATTERN_LA_LEG_WIDTH, +LEG_PATTERN_LA_LEG_WIDTH));
    candidate_FS_pattern.resize(3);

    template_SL_pattern.clear();
    template_SL_pattern.push_back(Point2(0, 0));
    template_SL_pattern.push_back(Point2(LEG_PATTERN_SL_LEG_WIDTH, 0));
    candidate_SL_pattern.resize(2);

    // publishers
    _laser_sub = _nh_public.subscribe(laser_src_topic, 1, &LegPPLP::scan_callback, this);
#ifdef PUBLISH_MARKER
    _marker_pub = _nh_private.advertise<visualization_msgs::Marker>("marker", 1);
#endif // PUBLISH_MARKER
    ROS_WARN
        ("leg_pplp:This executable subscribes to a laser scan '%s', "
         "and find leg patterns in them. It publishes PeoplePoseList results to '%s'.",
         _laser_sub.getTopic().c_str(), get_ppl_topic().c_str());
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    _laser_sub.shutdown();
#ifdef PUBLISH_MARKER
    _marker_pub.shutdown();
#endif // PUBLISH_MARKER
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_edge_marker() {
#ifdef PUBLISH_MARKER
    marker_msg.colors.clear();
    marker_msg.colors.reserve(2 * edges.size());
    std_msgs::ColorRGBA color;
    color.a = 1;
    std::vector<Point2> marker_pts;
    for (unsigned int edge_idx = 0; edge_idx < edges.size(); ++edge_idx) {
      marker_pts.push_back(edges[edge_idx].p0);
      marker_pts.push_back(edges[edge_idx].p1);
      // left = blue, right = red
      color.b = (edges[edge_idx].type == Edge::LEFT_EDGE);
      color.r = 1 - color.b;
      marker_msg.colors.push_back(color);
      marker_msg.colors.push_back(color);
    } // end loop edge_idx
    marker_utils::list_points2_as_primitives
        (marker_msg, marker_pts, "edges_ends", 0.1, // z
         0.02, // size
         1, 0, 0, 1,
         marker_msg.header.frame_id, visualization_msgs::Marker::LINE_LIST);
    _marker_pub.publish(marker_msg);
    //  ROS_INFO_THROTTLE(1, "Publishing a message with %i colours, and %i points",
    //                    marker_msg.colors.size(), marker_msg.points.size());
#endif // PUBLISH_MARKER
  } // end publish_edge_marker();

  //////////////////////////////////////////////////////////////////////////////

  /*!
 \param E
    the concerned edges will be removed (the vector is changed)
 \param out
    the position of the found LA legs
*/
  void find_LA_patterns(std::vector<Edge> & E,
                        std::vector<Point2> & out) {
    // printf("find_LA_patterns()\n");
    // Timer timer;
    out.clear();
    // e_m = first left edge of E
    unsigned int m = 0;
    while (m < E.size() && E[m].type != Edge::LEFT_EDGE)
      ++m;

    while (m + 3 < E.size()) {
      // printf("m:%i\n", m);
      // left leg matching pattern?
      if (E[m].type == Edge::LEFT_EDGE && E[m + 1].type == Edge::RIGHT_EDGE) {
        unsigned int n = m + 2;
        while (n + 1 < E.size()) {
          //printf("m:%i, n:%i\n", m, n);
          // right leg matching pattern?
          if (E[n].type == Edge::LEFT_EDGE && E[n + 1].type == Edge::RIGHT_EDGE) {
            candidate_LA_pattern[0] = E[m]    .p1;
            candidate_LA_pattern[1] = E[m + 1].p0;
            candidate_LA_pattern[2] = E[n]    .p1;
            candidate_LA_pattern[3] = E[n + 1].p0;
            double pattern_dist = geometry_utils::distance_patterns
                                  (template_LA_pattern, candidate_LA_pattern, LEG_PATTERN_LA_MAX_DIST);
            //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: possible LA pattern at indices %i, %i, %i, %i\n",
            //         pattern_dist, m, m + 1, n, n + 1);
            if (pattern_dist < LEG_PATTERN_LA_MAX_DIST) {
              //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: we found a LA pattern at indices %i, %i, %i, %i",
              //                    pattern_dist, m, m + 1, n, n + 1);
              // store the position of the pattern
              out.push_back(Point2(.5 * (candidate_LA_pattern[1].x + candidate_LA_pattern[2].x),
                  .5 * (candidate_LA_pattern[1].y + candidate_LA_pattern[2].y)));
              // remove e_m, e_m+1, e_n, e_n+1 from E (in decreasing order)
              E.erase(E.begin() + n, E.begin() + n + 2);
              E.erase(E.begin() + m, E.begin() + m + 2);
              --m; // because E[m] is now different and m will be incremented!
              break;
            } // end if dist < thres
          } // end if right leg pattern
          ++n;
        } // end while (!found && n + 1 < E.size())
      } // end if left leg matching pattern
      ++m;
    } // end while (m + 3 < in.size)
    // timer.printTime("for find_LA_patterns();");
  } // end find_LA_patterns();

  //////////////////////////////////////////////////////////////////////////////

  /*!
 \param E
    the concerned edges will be removed (the vector is changed)
 \param out
    the position of the found FS legs
*/
  void find_FS_patterns(std::vector<Edge> & E,
                        std::vector<Point2> & out) {
    // printf("find_FS_patterns()\n");
    // Timer timer;
    out.clear();
    // e_m = first left edge of E
    unsigned int m = 0;
    while (m < E.size() && E[m].type != Edge::LEFT_EDGE)
      ++m;

    while (m + 2 < E.size()) {
      // printf("m:%i\n", m);
      // left leg matching pattern?
      if ((E[m].type == Edge::LEFT_EDGE
           && E[m + 1].type == Edge::LEFT_EDGE
           && E[m + 2].type == Edge::RIGHT_EDGE)
          || (E[m].type == Edge::LEFT_EDGE
              && E[m + 1].type == Edge::RIGHT_EDGE
              && E[m + 2].type == Edge::RIGHT_EDGE)) {
        double pattern_dist;
        // the point in the middle depends if it is a LLR or LRR config
        candidate_FS_pattern[0] = E[m]    .p1;
        candidate_FS_pattern[2] = E[m + 2].p0;
        if (E[m + 1].type == Edge::LEFT_EDGE) { // LLR -> p1, p1, p0
          candidate_FS_pattern[1] = E[m + 1].p1;
          pattern_dist = geometry_utils::distance_patterns
                         (template_FS_LLR_pattern, candidate_FS_pattern, LEG_PATTERN_FS_MAX_DIST);
          //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: possible LLR FS pattern at indices %i, %i, %i\n",
          //         pattern_dist, m, m + 1, m + 2);
        }
        else { // LRR -> p1, p0, p0
          candidate_FS_pattern[1] = E[m + 1].p0;
          pattern_dist = geometry_utils::distance_patterns
                         (template_FS_LRR_pattern, candidate_FS_pattern, LEG_PATTERN_FS_MAX_DIST);
          //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: possible LRR FS pattern at indices %i, %i, %i\n",
          //         pattern_dist, m, m + 1, m + 2);
        }

        if (pattern_dist < LEG_PATTERN_FS_MAX_DIST) {
          //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: we found a FS pattern at indices %i, %i, %i",
          //                    pattern_dist, m, m + 1, m + 2);
          // store the position of the pattern
          out.push_back(candidate_FS_pattern[1]);
          // remove e_m, e_m+1, e_m+2 from E
          E.erase(E.begin() + m, E.begin() + m + 3);
          --m; // because E[m] is now different and m will be incremented!
        } // end if dist < thres
      } // end if left leg matching pattern
      ++m;
    } // end while (m + 2 < in.size)
    // timer.printTime("for find_FS_patterns();");
  } // end find_FS_patterns();

  //////////////////////////////////////////////////////////////////////////////

  /*!
 \param E
    the concerned edges will be removed (the vector is changed)
 \param out
    the position of the found SL legs
*/
  void find_SL_patterns(std::vector<Edge> & E,
                        std::vector<Point2> & out) {
    // printf("find_SL_patterns()\n");
    // Timer timer;
    out.clear();
    // e_m = first left edge of E
    unsigned int m = 0;
    while (m < E.size() && E[m].type != Edge::LEFT_EDGE)
      ++m;

    while (m + 1 < E.size()) {
      // printf("m:%i\n", m);
      // left leg matching pattern?
      if (E[m].type == Edge::LEFT_EDGE
          && E[m + 1].type == Edge::RIGHT_EDGE) {
        candidate_SL_pattern[0] = E[m]    .p1;
        candidate_SL_pattern[1] = E[m + 1].p0;
        double pattern_dist = geometry_utils::distance_patterns
                              (template_SL_pattern, candidate_SL_pattern, LEG_PATTERN_SL_MAX_DIST);

        if (pattern_dist < LEG_PATTERN_SL_MAX_DIST) {
          //  ROS_INFO_THROTTLE(1, "pattern_dist=%g: we found a SL pattern at indices %i, %i",
          //                    pattern_dist, m, m + 1);
          // store the position of the pattern
          out.push_back(Point2(.5 * (candidate_SL_pattern[1].x + candidate_SL_pattern[0].x),
              .5 * (candidate_SL_pattern[1].y + candidate_SL_pattern[0].y)));
          // remove e_m, e_m+1, e_m+2 from E
          E.erase(E.begin() + m, E.begin() + m + 2);
          --m; // because E[m] is now different and m will be incremented!
        } // end if dist < thres
      } // end if left leg matching pattern
      ++m;
    } // end while (m + 2 < in.size)
    // timer.printTime("for find_SL_patterns();");
  } // end find_SL_patterns();

  //////////////////////////////////////////////////////////////////////////////

  void scan_callback(const sensor_msgs::LaserScanConstPtr & scan_ptr) {
    // ROS_INFO_THROTTLE(1, "scan_callback()");

    // 1) data preprocessing
    scan_preprocessed = *scan_ptr;
    apply_local_minimization(scan_ptr->ranges, scan_preprocessed.ranges,
                             scan_ptr->ranges.size(), local_minimization_window_size);

    // 2) detection of vertical edges
    edges.clear();
    for (unsigned int range_idx = 1; range_idx < scan_preprocessed.ranges.size(); ++range_idx) {
      // check if distance between readings is high
      if (fabs(scan_preprocessed.ranges[range_idx] - scan_preprocessed.ranges[range_idx - 1])
          > edge_distance_threshold) {
        // push current segment
        Edge e;
        e.p0 = point_from_polar
               (scan_preprocessed.ranges[range_idx - 1],
            scan_preprocessed.angle_min +
            (range_idx - 1) * scan_preprocessed.angle_increment);
        e.p1 = point_from_polar
               (scan_preprocessed.ranges[range_idx],
                scan_preprocessed.angle_min +
                range_idx  * scan_preprocessed.angle_increment);
        if (scan_preprocessed.ranges[range_idx - 1] > scan_preprocessed.ranges[range_idx])
          e.type = Edge::LEFT_EDGE;
        else
          e.type = Edge::RIGHT_EDGE;
        edges.push_back(e);
      }
    } // end loop range_idx
    // ROS_INFO_THROTTLE(1, "Found %i edges", edges.size());

    unsigned int edges_size_before = edges.size();
    connected_adjacent_edges(edges);
//    ROS_INFO_THROTTLE(1, "Connecting adjacent edges: %i -> %i edges",
//                      edges_size_before, edges.size());

#ifdef PUBLISH_MARKER
    // need to publish edges before finding patterns
    // because the edges in patterns are erased from edges vector
    publish_edge_marker();
#endif // PUBLISH_MARKER

    std::vector<Point2> LA_patterns, FS_patterns, SL_patterns;
    // LA
    find_LA_patterns(edges, LA_patterns);
    // FS
    find_FS_patterns(edges, FS_patterns);
    // SL
    find_SL_patterns(edges, SL_patterns);
    ROS_INFO_THROTTLE(1, "Found patterns : %i LA, %i FS, %i SL",
                      LA_patterns.size(), FS_patterns.size(), SL_patterns.size());

    // timer.printTime("scan_callback();");

    // format old message
    unsigned int n_people = LA_patterns.size() + FS_patterns.size() + SL_patterns.size();
    // build new message
    _ppl.header = scan_ptr->header;
    _ppl.method = "leg_pplp";
    _ppl.poses.clear();
    _ppl.poses.reserve(n_people);
    for (unsigned int head_idx = 0; head_idx < n_people; ++head_idx) {
      people_msgs::PeoplePose people_pose;
      people_pose.header = _ppl.header; // copy header
      people_pose.person_name = StringUtils::cast_to_string(head_idx);
      people_pose.confidence = 1;
      people_pose.std_dev = .1;
      // set pose
      geometry_msgs::Pose* new_pose = &(people_pose.head_pose);
      new_pose->orientation = tf::createQuaternionMsgFromYaw(0);
      if (head_idx < LA_patterns.size())
        pt_utils::copy2(LA_patterns[head_idx],
                        new_pose->position);
      else if (head_idx < LA_patterns.size() + FS_patterns.size())
        pt_utils::copy2(FS_patterns[head_idx - LA_patterns.size()],
            new_pose->position);
      else
        pt_utils::copy2(SL_patterns[head_idx - LA_patterns.size() - FS_patterns.size()],
            new_pose->position);
      new_pose->position.z = 1.7;

      // add it
      _ppl.poses.push_back(people_pose);
    } // end loop head_idx
    // publish message
    publish_PPL(_ppl);

#ifdef PUBLISH_MARKER
    marker_msg.header.frame_id = scan_ptr->header.frame_id;
    marker_utils::list_points2_as_primitives
        (marker_msg, LA_patterns, "LA_patterns", 0.1, // z
         0.2 /* size */ , 1, 0, 0, 1, marker_msg.header.frame_id);
    _marker_pub.publish(marker_msg);
    marker_utils::list_points2_as_primitives
        (marker_msg, FS_patterns, "FS_patterns", 0.1, // z
         0.2 /* size */ , 0, 1, 0, 1, marker_msg.header.frame_id);
    _marker_pub.publish(marker_msg);
    marker_utils::list_points2_as_primitives
        (marker_msg, SL_patterns, "SL_patterns", 0.1, // z
         0.2 /* size */ , 0, 0, 1, 1, marker_msg.header.frame_id);
    _marker_pub.publish(marker_msg);
#endif // PUBLISH_MARKER

  } // end scan_callback();

  //////////////////////////////////////////////////////////////////////////////
private:

  //////////////////////////////////////////////////////////////////////////////
  // params
  std::string laser_src_topic;
  double edge_distance_threshold;
  int local_minimization_window_size;
  // data
  ros::Subscriber _laser_sub;
  sensor_msgs::LaserScan scan_preprocessed;
  std::vector<Edge> edges;

  //! where we will put the different pattern
  std::vector<Point2> template_LA_pattern;
  std::vector<Point2> template_FS_LLR_pattern;
  std::vector<Point2> template_FS_LRR_pattern;
  std::vector<Point2> template_SL_pattern;
  //! where we found the different patterns
  std::vector<Point2> candidate_LA_pattern;
  std::vector<Point2> candidate_FS_pattern;
  std::vector<Point2> candidate_SL_pattern;

#ifdef PUBLISH_MARKER
  visualization_msgs::Marker marker_msg;
  ros::Publisher _marker_pub;
#endif // PUBLISH_MARKER

  //! ROS message to share results
  people_msgs::PeoplePoseList _ppl;
}; // end class LegPPLP

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_pplp");
  LegPPLP skill;
  ros::spin();
  return 0;
}
