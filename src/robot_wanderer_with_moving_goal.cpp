/*!
  \file         robot_wanderer_with_moving_goal.cpp
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

  This node enables the following and tracking of a moving goal.
  The free space is obtained thanks to the local costmap
  delivered by the move_base componnet.

  Many parameters enable to configure the behaviour of the robot.
  Most notably minimum and maximum speed

\section Parameters
  - \b robot_frame_id
    [string] (default: "/base_link")
    The frame of the robot, normally attached to the base link.

  - \b static_frame_id
    [string] (default: "/odom")
    A static frame for the world, like /map or /odom .

  - \b inflated_obstacles_topic
    [string] (default: "/move_base/local_costmap/inflated_obstacles")
    Where to get the costmap in the local frame.

  - \b goal_pt_topic
    [string] (default: "~moving_goal")
    Where will be received the actualizations for the moving goal,
    published by other nodes.

  - \b stop_tracking_order_topic
    [string] (default: "~stop_tracking_order")
    Where will be received the stop orders,
    published by other nodes.

  - \b cmd_vel_topic
    [string] (default: "/cmd_vel")
    Where to publish the speed orders that will move the base.

  - \b min_vel_lin
    [double] m.s-1 (default: .1)
    The minimum linear speed.

  - \b max_vel_lin
    [double] m.s-1 (default: .3)
    The maximum linear speed.

  - \b max_vel_ang
    [double] rad.s-1 (default: .5)
    The maximum absolute angular speed. The minimum angular speed is 0.
    The search domain is then: [-max_vel_ang .. max_vel_ang]

  - \b min_rotate_on_place_speed
    [double] rad.s-1 (default: fabs(max_vel_ang) * 2 / 3)
    The minimum on place rotation speed,
    for when the robot is close enough but not centered to the goal.

  - \b max_rotate_on_place_speed
    [double] rad.s-1 (default: fabs(max_vel_ang))
    The maximum on place rotation speed,
    for when the robot is close enough but not centered to the goal.

  - \b min_goal_distance
    [double] m (default: .6)
    The minimum distance between the robot center and the goal center
    When this distance is reached, the robot stopped.
    (do not forget that it includes the robot radius!)

  - \b max_goal_angle
    [double] m (default: .2)
    The maximum angle allowed with the goal.
    If the nose of the robot is pointing to a direction making an angle
    higher than this value, it will rotate on place.

  - \b _speed_recomputation_timeout
    [double] m (default: 1)
    Maximum time before we choose some other speed when tracking

  - \b no_goal_timeout
    [double] m (default: 2)
    maximum time before stopping the robot when no new goal received

  - \b time_pred
    [double] sec (default: 5)
    The time of planning, and time needed to arrive at the goal.

  - \b max_tries
    [int] (default: 1000)
    The time of planning, and time needed to arrive at the goal.
    the number of tried trajectories

  - \b goal_distance_topic
    [string] (default: "/moving_goal_distance")
    Where this node will send the distances to the moving goal.

\section Subscriptions
  - \b {inflated_obstacles_topic}
  [nav_msgs/GridCells]
  Where we get the local costmap.

  - \b {goal_pt_topic}
    [geometry_msgs/PoseStamped]
    Where to get the actualisations for the goal.

  - \b {stop_tracking_order_topic}
    [std_msgs/Empty]
    To stop the tracking

\section Publications
  - \b {cmd_vel_topic}
    [geometry_msgs::Twist]
    The orders sent to the base.

  - \b "/robot_wanderer_traj"
    [visualization_msgs::Marker]
    A marker to visualize the current trajectory.

  - \b {goal_distance_topic}
    [std_msgs/Float64]
    The distance to the moving goal.

*/

// uncomment to make the robot talk about its state
#define USE_ETTS

// C
#include <signal.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
// ros utils





#include "vision_utils/timer.h"
#include "vision_utils/genetic.h"

#ifdef USE_ETTS
#include "vision_utils/nano_etts_api.h"
#endif // USE_ETTS

////////////////////////////////////////////////////////////////////////////////

// TYPEDEFS
typedef vision_utils::FooPoint2f Pt2;
struct Pose2 {
  Pt2 position;
  float yaw;
};

////////////////////////////////////////////////////////////////////////////////

template<typename T>
inline T clamp(T Value, T Min, T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

/*!
 * \struct SpeedOrder
 * A minimalistic structure for representing an order sent to a mobile base,
 * made of a linear and an angular speed.
 */
struct SpeedOrder {
  double vel_lin;
  double vel_ang;
  SpeedOrder() : vel_lin(0), vel_ang(0) {}
  SpeedOrder(const double & lin, const double & ang)
    : vel_lin(lin), vel_ang(ang) {}

  //////////////////////////////////////////////////////////////////////////////

  /*! clamp vel_ang to interval
   [-max_vel_ang, max_vel_ang]
    and vel_lin to interval
   [-max_lin_speed, -min_lin_speed] U [min_lin_speed, max_lin_speed] */
  void clamp_speed(const double & min_vel_lin_,
                   const double & max_vel_lin_,
                   const double & max_vel_ang_) {
    vel_ang = clamp(vel_ang, -max_vel_ang_, max_vel_ang_);
    vel_lin = clamp(vel_lin, min_vel_lin_, max_vel_lin_);
  } // end clamp();
}; // end class SpeedOrder

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class RobotWandererWithMovingGoal : public GeneticSolver<SpeedOrder> {
public:
  //! the time step simulation
  static const double DT = 0.2;
  enum Action {
    UNDEFINED = -1,
    KEEP_SAME_SPEED = 0,
    STOP = 1,
    ROTATE_ON_PLACE = 2,
    RECOMPUTE_SPEED = 3
  };

  ////////////////////////////////////////////////////////////////////////////////
  //class SpeedGeneticFinder : public GeneticSolver<SpeedOrder> {
  ////////////////////////////////////////////////////////////////////////////////

  //! the higher the better
  inline double fitness(const SpeedOrder & to_grade) {
    return - traj_grade(to_grade);
  } // end fitness()

  //////////////////////////////////////////////////////////////////////////////

  inline void crossover(const SpeedOrder & parent1,
                        const SpeedOrder & parent2,
                        SpeedOrder & crossed_son) {
    crossed_son.vel_ang = (parent1.vel_ang + parent2.vel_ang) / 2;
    crossed_son.vel_lin = (parent1.vel_lin + parent2.vel_lin) / 2;
    crossed_son.clamp_speed(_min_vel_lin, _max_vel_lin, _max_vel_ang);
  } // end crossover();

  //////////////////////////////////////////////////////////////////////////////

  /*!
      The mutation function.
      \arg mutation_rate
            between 0 (no randomness) and 1 (completely random)
     */
  inline void mutation(const SpeedOrder & parent,
                       const double & mutation_rate,
                       SpeedOrder & mutated_son) {
    mutated_son.vel_ang = parent.vel_ang +
                          mutation_rate * (drand48() * 2 - 1) * _max_vel_ang;
    mutated_son.vel_lin = parent.vel_lin +
                          mutation_rate * (drand48() * 2 - 1) * _max_vel_lin;
    mutated_son.clamp_speed(_min_vel_lin, _max_vel_lin, _max_vel_ang);
  } // end mutation();

  //////////////////////////////////////////////////////////////////////////////
  //}; // end class SpeedGeneticFinder
  //////////////////////////////////////////////////////////////////////////////

  RobotWandererWithMovingGoal() {
    // stop the robot if signal received
    //signal(SIGINT, stop_robot_and_exit);
    //signal(SIGTERM, stop_robot_and_exit);
    //signal(SIGKILL, stop_robot_and_exit);

    _was_stopped = true;
    _is_goal_active = false;

    //configure ROS
    ros::NodeHandle nh_public, nh_private("~");

    // get params
    _robot_frame_id = "/base_link";
    nh_private.param("robot_frame_id", _robot_frame_id, _robot_frame_id);
    _static_frame_id = "/odom";
    nh_private.param("static_frame_id", _static_frame_id, _static_frame_id);
    _inflated_obstacles_topic = "move_base/local_costmap/inflated_obstacles";
    nh_private.param("inflated_obstacles_topic",
                     _inflated_obstacles_topic,
                     _inflated_obstacles_topic);
    _moving_goal_topic = "moving_goal";
    nh_private.param("goal_pt_topic", _moving_goal_topic, _moving_goal_topic);
    _stop_tracking_order_topic = "stop_tracking_order";
    nh_private.param("stop_tracking_order_topic", _stop_tracking_order_topic, _stop_tracking_order_topic);
    _cmd_vel_topic = "cmd_vel";
    nh_private.param("cmd_vel_topic", _cmd_vel_topic, _cmd_vel_topic);
    _goal_distance_topic = "moving_goal_distance";
    nh_private.param("goal_distance_topic",
                     _goal_distance_topic, _goal_distance_topic);

    nh_private.param("min_vel_lin", _min_vel_lin, .1);
    nh_private.param("max_vel_lin", _max_vel_lin, .3);
    nh_private.param("max_vel_ang", _max_vel_ang, .5);
    _min_rotate_on_place_speed = fabs(_max_vel_ang) * 2. / 3;
    nh_private.param("min_rotate_on_place_speed",
                     _min_rotate_on_place_speed, _min_rotate_on_place_speed);
    _max_rotate_on_place_speed = fabs(_max_vel_ang);
    nh_private.param("max_rotate_on_place_speed",
                     _max_rotate_on_place_speed, _max_rotate_on_place_speed);
    nh_private.param("min_goal_distance", _min_goal_distance, .6);
    nh_private.param("max_goal_angle", _max_goal_angle, .2);
    nh_private.param("speed_recomputation_timeout",
                     _speed_recomputation_timeout, 1.);
    nh_private.param("no_goal_timeout", _no_goal_timeout, 2.);
    nh_private.param("time_pred", _time_pred, 5.);
    nh_private.param("max_tries", _max_tries, 1000);

    // publishers
    _marker_pub = nh_public.advertise<visualization_msgs::Marker>
                  ("robot_wanderer_traj", 1);
    _vel_pub = nh_public.advertise<geometry_msgs::Twist>
               (_cmd_vel_topic, 1);
    _goal_distance_pub = nh_public.advertise<std_msgs::Float64>
                         (_goal_distance_topic, 1);
    _etts_api.advertise();
    // subscribers
    _inflated_sub = nh_public.subscribe
                    (_inflated_obstacles_topic, 1,
                     &RobotWandererWithMovingGoal::inflated_obstacles_cb, this);
    _goal_sub = nh_public.subscribe
                (_moving_goal_topic, 1,
                 &RobotWandererWithMovingGoal::goal_cb, this);
    _stop_tracking_order_sub = nh_private.subscribe
                               (_stop_tracking_order_topic, 1,
                                &RobotWandererWithMovingGoal::stop_tracking_order_cb,
                                this);
    ROS_INFO("robot_wanderer_with_moving_goal: "
             "Getting inflated obstacles from '%s' and goal from '%s', "
             "emitting speed to '%s'. "
             "Recomputing speeds every %g s. Stopping if no goal during %g s",
             _inflated_sub.getTopic().c_str(),
             _goal_sub.getTopic().c_str(),
             _vel_pub.getTopic().c_str(),
             _speed_recomputation_timeout, _no_goal_timeout);
  } // end ctor

  ////////////////////////////////////////////////////////////////////////////////

  ~RobotWandererWithMovingGoal() {
    stop_robot_and_exit();
  }

  ////////////////////////////////////////////////////////////////////////////////

  void refresh() {
    if (!_is_goal_active)
      return;
    // determine if we need to keep on publishing orders
    if (_last_goal_timer.getTimeSeconds() > _no_goal_timeout) {
      ROS_INFO_THROTTLE(1, "_no_goal_timeout,  stopping emitting speed orders");
      stop_robot();
      _is_goal_active = false; // need to be after stop_robot();
    } // end if (_last_goal_timer.getTimeSeconds() > _no_goal_timeout)

    // publish the computed speed
    _out.linear.x = _current_order.vel_lin;
    _out.angular.z = _current_order.vel_ang;
    ROS_INFO_THROTTLE(1, "robot_wanderer_with_moving_goal: "
                         "Publishing _vel_lin:%g, _vel_ang:%g",
                      _current_order.vel_lin, _current_order.vel_ang);
    _vel_pub.publish(_out);
  } // end refresh()

protected:
  ////////////////////////////////////////////////////////////////////////////////

  /*!
  Say a sentence using the ETTS system.
  Only does it if the flag USE_ETTS is not commented.
 \param sentence
*/
  inline void say_sentence(const std::string & sentence) {
    printf("robot_wanderer_with_moving_goal:say_sentence('%s')\n", sentence.c_str());
#ifdef USE_ETTS
    _etts_api.say_text(sentence);
#endif // USE_ETTS
  } // end say_sentence();

  ////////////////////////////////////////////////////////////////////////////////

  inline void set_speed(const SpeedOrder & new_speed) {
    ROS_INFO_THROTTLE(1, "set_speed(lin:%g, ang:%g)",
                      new_speed.vel_lin, new_speed.vel_ang);
    _was_stopped = (new_speed.vel_lin == 0) && (new_speed.vel_ang == 0);
    _current_order = new_speed;
    _is_goal_active = true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*!
  stop the robot
*/
  inline void stop_robot() {
    set_speed(SpeedOrder());
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*!
  send a 0 speed to the robot and exit program
 \param param
    the exit code returned
*/
  void stop_robot_and_exit(int param = 0) {
    ROS_INFO_THROTTLE(1, "stop_robot_and_exit()");
    stop_robot();
    ros::shutdown();
    exit(param);
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! cb when a stop order is received
  void stop_tracking_order_cb(const std_msgs::EmptyConstPtr & msg) {
    // first time we stop, say something
    if (!_was_stopped) {
      ROS_INFO_THROTTLE(1, "There is no valid goal. Stopping the robot.");
      say_sentence("|en:There is no goal.");
    }
    stop_robot();
    _is_goal_active = false;
  } // end stop_tracking_order_cb();

  ////////////////////////////////////////////////////////////////////////////////

  /*!
 Evaluate how good a speed order is for reaching the goal.
 \param order
    the possible speed order
 \return float
    the lower the better.
    infinity if collision with the costmap,
    otherwise: fabs(min_goal_distance - the L2 distance to the goal)
*/
  inline float traj_grade(const SpeedOrder & order) {
    double goal_distance = vision_utils::trajectory_mark<Pt2>
                           (_current_robot_pose.position, // Pt2
                            _current_robot_pose.yaw, // float
                            _goal.position, // Pt2
                            _local_costmap, // nav_msgs::GridCells
                            _time_pred, DT,
                            order.vel_lin, order.vel_ang,
                            _traj_buffer);
    return goal_distance;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void recompute_speed() {
    vision_utils::Timer timer;
    //_speed_genetic_finder.run_algorithm();
    //SpeedOrder best_order = _speed_genetic_finder.get_best_element().element;
    run_algorithm();
    SpeedOrder best_order = get_best_element().element;
    ROS_INFO_THROTTLE(1, "Found the most suitable couple of speed in %g ms, "
                         "with a grade %g:"
                         "_vel_lin:%g, _vel_ang:%g",
                      timer.getTimeMilliseconds(),
                      //_speed_genetic_finder.get_best_element().grade,
                      get_best_element().grade,
                      best_order.vel_lin, best_order.vel_ang);
    set_speed(best_order);
  } // end recompute_speed();

  ////////////////////////////////////////////////////////////////////////////////

  void goal_cb(const geometry_msgs::PoseStampedConstPtr & pt3D) {
    //ROS_INFO_THROTTLE(1, "goal_cb()");

    // do nothing if the pt is empty
    if (pt3D->header.frame_id != _static_frame_id) {
      ROS_ERROR("The point frame_id '%s' is different from our static frame_id '%s'!",
                pt3D->header.frame_id.c_str(), _static_frame_id.c_str());
      stop_robot();
    }

    // reset the timer for the last time we saw a valid goal
    _last_goal_timer.reset();

    // make a copy of the goal
    vision_utils::copy2(pt3D->pose.position, _goal.position);
    _goal.yaw = tf::getYaw(pt3D->pose.orientation);
    //_goal = *pt3D;

    // get the position of the robot in the static frame
    geometry_msgs::PoseStamped robot_pose_robot_frame, robot_pose_static_frame;
    robot_pose_robot_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    robot_pose_robot_frame.header.frame_id = _robot_frame_id;
    //    robot_pose_robot_frame.header.stamp = ros::Time::now();
    robot_pose_robot_frame.header.stamp = pt3D->header.stamp;
    robot_pose_static_frame.header.frame_id = _static_frame_id;
    robot_pose_static_frame.header.stamp = pt3D->header.stamp;
    try {

      //    _tf_listener.transformPose(_static_frame_id,  ros::Time(0),
      //                                robot_pose_robot_frame,
      //                                _static_frame_id,
      //                                robot_pose_static_frame);

      _tf_listener.transformPose(_static_frame_id,
                                 robot_pose_robot_frame, robot_pose_static_frame);
    } catch (tf::ExtrapolationException e) {
      ROS_WARN_THROTTLE(2, "transform error:'%s'", e.what());
      return;
    }
    vision_utils::copy2(robot_pose_static_frame.pose.position,
                    _current_robot_pose.position);
    _current_robot_pose.yaw = tf::getYaw(robot_pose_static_frame.pose.orientation);
    //  ROS_INFO_THROTTLE(1, "Robot is in (%g, %g), yaw:%g in '%s'.",
    //                    _current_robot_pose.position.x, _current_robot_pose.position.y,
    //                    _current_robot_pose.yaw, _static_frame_id.c_str());


    // determine if we have reached the goal - 2D distance
    float curr_goal_distance =
        //vision_utils::distance_points
        hausdorff_distances::dist_L2
        (_current_robot_pose.position, _goal.position);
    float vector_to_goal_yaw =
        vision_utils::oriented_angle_of_a_vector
        (_goal.position - _current_robot_pose.position);
    float angle_to_goal = vector_to_goal_yaw - _current_robot_pose.yaw;
    if (angle_to_goal > M_PI)
      angle_to_goal -= M_PI * 2;
    else if (angle_to_goal < -M_PI)
      angle_to_goal += M_PI * 2;
    ROS_DEBUG_THROTTLE(1, "goal_distance:%g, angle_to_goal:%g degrees (%g -%g)",
                       curr_goal_distance, angle_to_goal * RAD2DEG,
                       vector_to_goal_yaw, _current_robot_pose.yaw);

    // publish distance to goal
    std_msgs::Float64 curr_goal_distance_msg;
    curr_goal_distance_msg.data = curr_goal_distance;
    _goal_distance_pub.publish(curr_goal_distance_msg);

    Action current_action = UNDEFINED;

    // stop going forward if we are too close
    if (curr_goal_distance < _min_goal_distance) {
      // not centered to goal => rotate on place
      if (fabs(angle_to_goal) > _max_goal_angle) {
        ROS_INFO_THROTTLE
            (1, "Close enough from goal (dist:%g < min_goal_distance:%g) "
                "but need to rotate: fabs(angle_to_goal:%g) > max_goal_angle:%g",
             curr_goal_distance, _min_goal_distance,
             fabs(angle_to_goal), _max_goal_angle);
        current_action = ROTATE_ON_PLACE;
      }
      // centered to goal => stop
      else {
        ROS_INFO_THROTTLE(1, "Close enough from goal (dist:%g < min_goal_distance:%g), "
                             " no need to rotate...",
                          curr_goal_distance, _min_goal_distance);
        current_action = STOP;
        if (!_was_stopped){
          ROS_INFO_THROTTLE(1, "Goal reached (dist:%g < min_goal_distance:%g) ! "
                               "Stopping.",
                            curr_goal_distance, _min_goal_distance);
        } // end if (!_was_stopped)
      } // end if (fabs(angle_to_goal) > max_goal_angle)
    } // end if (goal_distance < min_goal_distance)

    /*
   * determine if needed to recompute the trajectory
   */
    // do not recmopute if the speed is pretty good
    float curr_grade = traj_grade(_current_order);

    // recompute if collision coming
    if (current_action == UNDEFINED) {
      if (curr_grade > 1E3) { // traj_grade() return infinity if collision
        ROS_WARN_THROTTLE(1, "collision coming with this speed, recomputing speed.");
        current_action = RECOMPUTE_SPEED;
      }
    } // end if (current_action == UNDEFINED)

    //  if (current_action == UNDEFINED) {
    //    ROS_INFO_THROTTLE(1, "curr_grade:%f, min_goal_distance:%f",
    //                      curr_grade, min_goal_distance);
    //    if (curr_grade < 1.2 * min_goal_distance) {
    //      ROS_INFO_THROTTLE(1, "this speed is good, not changing");
    //      current_action = KEEP_SAME_SPEED;
    //    } // end
    //  } // end if (current_action == UNDEFINED)

    // recompute speed if the last ones are too old
    if (current_action == UNDEFINED) {
      if (_last_order_timer.getTimeSeconds() >
          _speed_recomputation_timeout) {
        ROS_INFO_THROTTLE(1, "_last_order_timer timeout (%f > timeout:%f)",
                          _last_order_timer.getTimeSeconds() ,
                          _speed_recomputation_timeout);
        current_action = RECOMPUTE_SPEED;
      }
      else {
        ROS_INFO_THROTTLE(1, "no timeout for _last_order_timer (%f < timeout:%f)",
                          _last_order_timer.getTimeSeconds() ,
                          _speed_recomputation_timeout);
        current_action = KEEP_SAME_SPEED;
      }
    } // end if (current_action == UNDEFINED)

    // normally we should not arrive here,
    // but in case of, recompute speed
    if (current_action == UNDEFINED) {
      ROS_WARN("Oh, oh, current_action == UNDEFINED. Decision tree uncomplete?");
      current_action = RECOMPUTE_SPEED;
    }

    /*
   * execute the current action
   */
    //ROS_INFO_THROTTLE(1, "current_action:%i", current_action);

    if (current_action == STOP) {
      if (!_was_stopped) // say something when we stop
        say_sentence("|en:Here I am.|es:He llegado.");

      // if yes, stop and return
      stop_robot();
    } // end if (current_action == STOP)

    // rotate on place when needed
    else if (current_action == ROTATE_ON_PLACE) {
      if (fabs(angle_to_goal) > _max_goal_angle) {
        // angle
        // (+) <--- 0 --> (-)
        double rotation_speed = fmin
                                (_max_rotate_on_place_speed,
                                 _min_rotate_on_place_speed * (1 + fabs(angle_to_goal) / _max_goal_angle)) / 2.f;
        if (angle_to_goal < 0)
          set_speed(SpeedOrder(0, -rotation_speed));
        else
          set_speed(SpeedOrder(0, +rotation_speed));
        return;
      } // end if (fabs(angle_to_goal) > max_goal_angle)
    } // end if (want_rotate_on_place)

    // recompute speed when needed
    else if (current_action == RECOMPUTE_SPEED) {
      recompute_speed();
    } // end if (current_action == RECOMPUTE_SPEED)

    else if (current_action == KEEP_SAME_SPEED)
    {
      ROS_INFO_THROTTLE(1, "We are happy with the speed _vel_lin:%g, _vel_ang:%g",
                        _current_order.vel_lin, _current_order.vel_ang);
    } // if (!current_action == KEEP_SAME_SPEED)

    // emit marker
    std::vector<Pt2> traj_xy;
    vision_utils::make_trajectory(_current_order.vel_lin, _current_order.vel_ang,
                                traj_xy, _time_pred, DT, 0, 0, 0);
    vision_utils::list_points2_as_primitives(_marker, traj_xy, "traj_xy",
                                             0.1, 0.03, 1, 0, 0, 1, _robot_frame_id);
    _marker.header.stamp = pt3D->header.stamp;
    _marker_pub.publish(_marker);
  } // end goal_cb();

  ////////////////////////////////////////////////////////////////////////////////

  void inflated_obstacles_cb(const nav_msgs::GridCellsConstPtr & msg) {
    //ROS_INFO_THROTTLE(1, "msg.header.frame_id:'%s'", msg->header.frame_id.c_str());
    if (msg->header.frame_id != _static_frame_id) {
      ROS_FATAL("The costmap frame_id '%s' is different from our static frame_id '%s'!",
                msg->header.frame_id.c_str(), _static_frame_id.c_str());
      stop_robot_and_exit(-1);
    }

    // keep a copy of the costmap
    _local_costmap = *msg;

#ifdef EMIT_LOCAL_COSTMAP_MARKER
    // publish cells centers
    std::vector<geometry_msgs::Point> cell_centers;
    for (unsigned int cell_idx = 0; cell_idx < msg->cells.size(); ++cell_idx) {
      const geometry_msgs::Point & curr_cell = msg->cells[cell_idx];
      cell_centers.push_back(geometry_msgs::Point(curr_cell.x, curr_cell.y));
    } // end loop cell_idx
    vision_utils::list_points2_as_primitives(_marker, cell_centers, "cell_centers",
                                             0.1, 0.05, 0, 1, 0, 1, static_frame_id);
    _marker_pub.publish(_marker);
#endif // EMIT_LOCAL_COSTMAP_MARKER
  } // end inflated_obstacles_cb();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //! the publishers
  ros::Publisher _marker_pub, _vel_pub;
  //! the vizu marker
  visualization_msgs::Marker _marker;
  ros::Subscriber _inflated_sub, _goal_sub, _stop_tracking_order_sub;


  /*
    * Parameters of the robot
    */
  std::string _static_frame_id, _robot_frame_id;
  //! the maximal velocities, m/s or rad/s
  double _min_vel_lin, _max_vel_lin, _max_vel_ang;
  double _min_rotate_on_place_speed, _max_rotate_on_place_speed;

  //! the current velocities, m/s or rad/s
  //SpeedGeneticFinder _speed_genetic_finder;
  SpeedOrder _current_order;
  bool _was_stopped;
  //! a flag to determine when to emit the speed orders
  bool _is_goal_active;
  //! the current pose of the robot
  Pose2 _current_robot_pose;

  // voice
#ifdef USE_ETTS
  NanoEttsApi _etts_api;
#endif // USE_ETTS

  //! the costmap receiving
  std::string _inflated_obstacles_topic;
  nav_msgs::GridCells _local_costmap;
  //#define EMIT_LOCAL_COSTMAP_MARKER

  //! where to get the goal
  std::string _moving_goal_topic;
  //! where to get the stop orders
  std::string _stop_tracking_order_topic;
  //! the current goal
  Pose2 _goal;

  /*! The minimum distance between the robot center and the goal center
      When this distance is reached, the robot stops.
      (do not forget that it includes the robot radius!) */
  double _min_goal_distance, _max_goal_angle;
  std::string _cmd_vel_topic;

  //! where to get the goal distance
  std::string _goal_distance_topic;
  ros::Publisher _goal_distance_pub;

  tf::TransformListener _tf_listener;
  //! timer since last computed speed
  Timer _last_order_timer;
  //! maximum time before we choose some other speed when tracking
  double _speed_recomputation_timeout;
  //! timer since last goal seen
  Timer _last_goal_timer;
  /*! maximum time before stopping the robot when no new goal received
    (should be higher than speed_recomputation_timeout) */
  double _no_goal_timeout;

  /*
    * Parameters for the simulation
    */
  //! the forecast time in seconds
  double _time_pred;
  //! the number of tried trajectories
  int _max_tries;
  std::vector<Pt2> _traj_buffer;
  geometry_msgs::Twist _out;
}; // end class RobotWandererWithMovingGoal

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_wanderer_with_moving_goal");
  RobotWandererWithMovingGoal node;
  // spin;
  ros::Rate rate(5);
  while(ros::ok()) {
    node.refresh();
    ros::spinOnce();
    rate.sleep();
  } // end while ok()
  return 0;
}
