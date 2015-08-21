/*!
  \file         fast_cluster_detector.cpp
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

\class FastClusterDetector
\brief A fast cluster detector.

\section Parameters
  - \b tracking_timeout
    [double, seconds] (default: 2.0)
    the time in seconds for declaring the object as lost when not detected.

  - \b min_cluster_size
    [double, ratio of the image size] (default: 2E-3 = around 150 pixels for 320x240)
    the minimum size of a cluster to be kept in percents of the original image.

  - \b max_cluster_size
    [double, ratio of the image size] (default: .5 = half of the image)
    the minimum size of a cluster to be kept in percents of the original image.

  - \b camera_info_topic
    [string] (default:/camera/depth_registered/camera_info)
    where to get info about the camera

  - \b max_mark_for_assoc
    [double] (default:1.0)
    the recognition is not valid when the mark between comp
    and matching object gets higher

  - \b mark_ratio_bbox_to_d22
    [double] (default:1.0)
    the higher the factor,
    the smaller the bbox grade and so
    the more important D22 and less the bbox similarity

\section Subscriptions
  - \b depth
    [sensor_msgs/Image]
    The depth image

  - \b {camera_info_topic}
    [sensor_msgs/CameraInfo]
    The model of the camera giving the depth images

 - \b "~set_tracking_seed"
      [geometry_msgs::PoseStamped]
      A new seed (origin) for the tracked point.

\section Publications
  - \b fast_cluster_functions::TRACKED_OBJECT_POSE_TOPIC
    [geometry_msgs::PoseStamped]
    The pose of the object being tracked.

  - \b "~status"
    [int8]
    A marker to show the cluster closest to the robot and the tracked cluster.

 - \b "~tracking_marker"
      [visualization_msgs::Marker]
      A marker to show the cluster closest to the robot and the tracked cluster.

*/

#include "vision_utils/utils/error.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4
#include "opencv2/photo/photo.hpp" // for versions 2.4 and +
#endif // CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4
#include <cv_bridge/cv_bridge.h>
// AD
#include "vision_utils/utils/debug_utils.h"
#include "vision_utils/utils/timer.h"
#include "vision_utils/disjoint_sets2.h"
#include "vision_utils/border_remover.h"
#include "vision_utils/color_utils.h"
#include "vision_utils/drawing_utils.h"
#include "vision_utils/value_remover.h"
#include <followme_laser/TrackingStatus.h>
#include <followme_laser/SetTrackingSeed.h>
// people_msgs
#include "vision_utils/cv_conversion_float_uchar.h"

#include "people_detection_vision/connected_components_matcher.h"
#include "people_detection_vision/fast_cluster_functions.h"
#include "vision_utils/utils/marker_utils.h"

//#define TIMER_ON

#define TRACK_BAR_SCALE_FACTOR 25.f

class FastClusterDetector {
public:
  typedef ConnectedComponentsMatcher::ObjectName ObjectName;

  //! true to show windows
  static const bool DISPLAY = true;
  /*! true to publish the objects names image on the topic
  fast_cluster_functions::OBJECTS_NAMES_IMAGE_TOPIC .
  It is for instance needed for the selector. */
  static const bool PUBLISH_OBJECTS_NAMES_IMG = false;
  /*! true to publish the GUI image on the topic /fast_cluster_detector_img_gui .
    It can be used for debug. */
  static const bool PUBLISH_GUI_IMG = false;
  /*! true to publish the objects names image on the topic
  fast_cluster_functions::COMPONENTS_ILLUS_TOPIC .
    It can be used for debug.*/
  static const bool PUBLISH_COMPONENTS_ILLUS = true;
  //! true to write on disk images
  static const bool SAVE_IMAGES = false;

  ////////////////////////////////////////////////////////////////////////////////

  enum NaNRemovalMethod {
    _VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION = 0,
    _VALUE_REMOVAL_METHOD_INPAINT = 1,
    _VALUE_REMOVAL_METHOD_AVERAGE_BORDER = 2
  }; // end NaNRemovalMethod

  //! the minimum size of a cluster to be kept in percents of the original image
  double min_cluster_size;
  double max_cluster_size;

  // these factors are multiplied by TRACK_BAR_SCALE_FACTOR (25) in the GUI
  static const double CANNY_PARAM1_DEFAULT_VAL = 0.88; // 1.3 // 2.0
  static const double CANNY_PARAM2_DEFAULT_VAL = 1.48; // 2.0 // 3.2

  //! the size of the kernel used to try to close contours (pixels)
  static const int MORPH_OPEN_KERNEL_SIZE = 3;

  std::string _static_frame;

  //! the time in seconds for declaring the object as lost when not detected
  double tracking_timeout;

  //////////////////////////////////////////////////////////////////////////////

  FastClusterDetector() :
    nh_private("~")
  {
    _current_status = followme_laser::TrackingStatus::NO_TARGET;

    // get params
    nh_private.param("tracking_timeout", tracking_timeout, 2.);
    // around 150 pixels for 320x240
    nh_private.param("min_cluster_size", min_cluster_size, 2E-3);
    // half of the image
    nh_private.param("max_cluster_size", max_cluster_size, .5);
    std::string camera_info_topic = "camera/depth_registered/camera_info";
    nh_private.param("camera_info_topic", camera_info_topic, camera_info_topic);

    double max_mark_for_assoc, mark_ratio_bbox_to_d22;
    nh_private.param("max_mark_for_assoc", max_mark_for_assoc, 1.0);
    nh_private.param("mark_ratio_bbox_to_d22", mark_ratio_bbox_to_d22, 1.0);


    // default NaN handling
    _nan_removal_method = _VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION;

    // set the threshold for matcher
    matcher.set_max_mark_for_assoc(max_mark_for_assoc);
    matcher.set_mark_ratio_bbox_to_d22(mark_ratio_bbox_to_d22);

    // status publisher
    _status_pub= nh_private.advertise<followme_laser::TrackingStatus>
        ("status", 1);

    // subscribe to the image message
    image_transport::ImageTransport it(nh_public);
    _depth_image_subscriber = it.subscribe
        ("depth",
         1,
         &FastClusterDetector::image_callback,
         this);

    if (PUBLISH_GUI_IMG)
      _objects_img_gui_pub = it.advertise(fast_cluster_functions::GUI_TOPIC, 1);
    if (PUBLISH_COMPONENTS_ILLUS)
      _components_illus_pub =
          it.advertise(fast_cluster_functions::COMPONENTS_ILLUS_TOPIC, 1);
    if (PUBLISH_OBJECTS_NAMES_IMG)
      _objects_names_img_pub =
          it.advertise(fast_cluster_functions::OBJECTS_NAMES_IMAGE_TOPIC, 1);
    _objects_nb_pub = nh_public.advertise<std_msgs::Int32>
        (fast_cluster_functions::OBJECTS_NB_TOPIC, 1);

    // reprojection
    ROS_INFO("Waiting for CameraInfo on '%s'", camera_info_topic.c_str());
    _cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
        (camera_info_topic, ros::Duration(3));
    if (!_cam_info) {
      ROS_FATAL("Could not get CameraInfo on '%s'. "
                "Dying in horrible pain...", camera_info_topic.c_str());
      ros::shutdown();
    }
    _cam_model.fromCameraInfo(_cam_info);
    _camera_frame = _cam_info->header.frame_id;
    _static_frame = "/odom";

    // tracking publishers and subscribers
    _tracked_object_coordinates_subscriber = nh_public.subscribe
        (fast_cluster_functions::TRACKED_OBJECT_COORDINATES_TOPIC, 1,
         &FastClusterDetector::tracked_object_coordinates_callback, this);
    _tracked_object_pose_publisher = nh_public.advertise<geometry_msgs::PoseStamped>
        (fast_cluster_functions::TRACKED_OBJECT_POSE_TOPIC, 1);
    _stop_tracking_order_publisher = nh_public.advertise<std_msgs::Empty>
        ("robot_wanderer_with_moving_goal/stop_tracking_order", 1);
    _tracked_object_name = fast_cluster_functions::NO_OBJECT;

    // tracking seed subscriber
    _tracking_seed_srv = nh_private.advertiseService
        ("set_tracking_seed", &FastClusterDetector::tracking_seed_callback, this);

    // marker
    _marker_pub = nh_private.advertise<visualization_msgs::Marker>
        ("tracking_marker", 1);
    marker_utils::make_header(_marker, visualization_msgs::Marker::SPHERE,
                              "fast_cluster_tracked_object",
                              0.2,   0, 0, 0, 1,  _static_frame);


    // configure the GUI for Canny parameters
    if (DISPLAY)
      cv::namedWindow("img_gui");
    canny_tb1_value = CANNY_PARAM1_DEFAULT_VAL * TRACK_BAR_SCALE_FACTOR;
    canny_tb2_value = CANNY_PARAM2_DEFAULT_VAL * TRACK_BAR_SCALE_FACTOR;
    if (DISPLAY) {
      cv::createTrackbar("canny_param1", "img_gui", &canny_tb1_value, 100);
      cv::createTrackbar("canny_param2", "img_gui", &canny_tb2_value, 100);
    } // end if (DISPLAY)
  }

  //////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  void image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
    //maggieDebug2("image_callback()");

    _timer.reset();
    _img_msg_timestamp = img_msg->header.stamp;

    // conversion to cv::Mat
    // cf
    // http://www.ros.org/wiki/cv_bridge/Tutorials/
    // UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    try {
      //_bridge_img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      _bridge_img_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & img_ptr = _bridge_img_ptr->image;
    //cv::imshow("img_ptr", img_ptr);

    //image_utils::print_random_pts_float<float>(img_ptr, 10);
    //image_utils::print_random_pts_int<uchar>(img_ptr, 10);
    // ...
    image_utils::convert_float_to_uchar(img_ptr, img_uchar, src_float_clean_buffer,
                                        alpha_trans, beta_trans);
    //cv::imshow("img_uchar", img_uchar);

    //print_random_pts_int<uchar>(img_uchar, 10);
    //    maggieDebug2("nonZero:%f",
    //                 1.f *cv::countNonZero(img_uchar) / (img_uchar.cols *img_uchar.rows));

#ifdef TIMER_ON
    _timer.printTime("after remapping");
#endif // TIMER_ON

    /*
     *remove NaN from input image
     */
    if (_nan_removal_method == _VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION) {
      // left value propagation
      img_uchar.copyTo(img_uchar_with_no_nan);
      image_utils::remove_value_left_propagation<uchar>(img_uchar_with_no_nan, image_utils::NAN_UCHAR);
    } // end if _VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION

    else if (_nan_removal_method == _VALUE_REMOVAL_METHOD_INPAINT) {
      // make inpaint
      img_uchar.copyTo(inpaint_mask);
      cv::threshold(img_uchar, inpaint_mask, 0, 255, cv::THRESH_BINARY_INV);
      cv::inpaint(img_uchar, inpaint_mask, img_uchar_with_no_nan, 5, cv::INPAINT_NS);
    } // en if _VALUE_REMOVAL_METHOD_INPAINT

    else if (_nan_removal_method == _VALUE_REMOVAL_METHOD_AVERAGE_BORDER) {
      // remove border
      roi = image_utils::remove_border(img_uchar, img_uchar_with_no_nan,
                                       image_utils::NAN_UCHAR, 0.5);
    } // end if _VALUE_REMOVAL_METHOD_AVERAGE_BORDER

    /*
     *edge detection
     */
    // canny
    canny_param1 = 1.f *canny_tb1_value / TRACK_BAR_SCALE_FACTOR;
    canny_param2 = 1.f *canny_tb2_value / TRACK_BAR_SCALE_FACTOR;
    maggieDebug3("canny_param1:%g, canny_param2:%g, alpha_trans:%g",
                 canny_param1, canny_param2, alpha_trans);
    cv::Canny(img_uchar_with_no_nan, edges,
              alpha_trans *canny_param1, alpha_trans *canny_param2);
    //    cv::Scalar mean = cv::mean(edges);
    //    cv::minMaxLoc(edges, &minVal, &maxVal);
    //    maggiePrint("min:%f, max:%f, mean:%f", minVal, maxVal, mean[0]);

    // harris corners
    //    cv::cornerHarris(img_ptr, edges, 3, 3, 0.01);
    //    print_random_pts<float>(edges, 10);

    //    double threshold= 0.0001;
    //    cv::threshold(edges, harrisCorners, threshold,255,cv::THRESH_BINARY);
    //    cv::imshow("harrisCorners", harrisCorners);

#ifdef TIMER_ON
    maggiePrint("Time after Canny (param1:%g, param2:%g):%g ms",
                canny_param1, canny_param2, _timer.time());
#endif // TIMER_ON

    /*invert the edges */
    cv::threshold(edges, edges_inverted, 128, 255, cv::THRESH_BINARY_INV);
    // close borders
    //image_utils::close_borders(edges_inverted_with_nan, (uchar) 255);
    //    cv::morphologyEx(edges_inverted, edges_inverted_opened,
    //                     cv::MORPH_OPEN,
    //                     cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    cv::erode(edges_inverted, edges_inverted_opened,
              cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    //timer.printTime("after close_borders()");

    /*
     *combine canny with nan
     */
    if (_nan_removal_method == _VALUE_REMOVAL_METHOD_AVERAGE_BORDER)
      cv::min(edges_inverted_opened, img_uchar(roi), edges_inverted_opened_with_nan);
    else
      cv::min(edges_inverted_opened, img_uchar, edges_inverted_opened_with_nan);
    cv::threshold(edges_inverted_opened_with_nan, edges_inverted_opened_with_nan,
                  image_utils::NAN_UCHAR, 255, cv::THRESH_BINARY);
#ifdef TIMER_ON
    _timer.printTime("after cv::min()");
#endif // TIMER_ON
    /*
     *connected components
     */
    _set.process_image(edges_inverted_opened_with_nan);
    _set.get_connected_components(edges_inverted_opened_with_nan.cols,
                                  _components_pts, _bounding_boxes);
#ifdef TIMER_ON
    unsigned int n_clusters_before = _components_pts.size();
    _timer.printTime("after connected components");
#endif // TIMER_ON

    // filter them by size
    unsigned int min_cluster_size_pixels =
        1.f *min_cluster_size *edges_inverted_opened.cols *edges_inverted_opened.rows;
    unsigned int max_cluster_size_pixels =
        1.f *max_cluster_size *edges_inverted_opened.cols *edges_inverted_opened.rows;
    for (unsigned int cluster_idx = 0;
         cluster_idx < _components_pts.size(); ++cluster_idx) {
      DisjointSets2::Comp*curr_comp = &(_components_pts.at(cluster_idx));
      uchar curr_val = img_uchar_with_no_nan.at<uchar>(curr_comp->front());
      //Bbox*curr_bbox = &(_bounding_boxes.at(cluster_idx));
      bool need_delete = false;
      // value = black -> remove
      if (curr_val == 0) // can be a image_utils::NAN_UCHAR or 0 from threshold
        need_delete = true;
      // min size test
      else if (curr_comp->size() < min_cluster_size_pixels)
        need_delete = true;
      // max test size
      else if (curr_comp->size() > max_cluster_size_pixels)
        need_delete = true;
      // other tests
      else {
        // no other test came to my mind :)
      } // end tests

      // now delete if needed
      if (need_delete) {
        _components_pts.erase(_components_pts.begin() + cluster_idx);
        _bounding_boxes.erase(_bounding_boxes.begin() + cluster_idx);
        // rewind the counter not to skip the next elt
        --cluster_idx;
        continue;
      } // end if need_delete
    } // end loop _components_pts
#ifdef TIMER_ON
    maggiePrint("Time after filtering:%g ms, filtering:%i -> %i clusters",
                _timer.time(),  n_clusters_before, _components_pts.size());
#endif // TIMER_ON
    /*
     *Matching
     */
    matcher.set_new_data(_components_pts, _bounding_boxes);
    matcher.match();
#ifdef TIMER_ON
    _timer.printTime("after matcher.match()");
#endif // TIMER_ON

    /*
     *tracking
     */
    track_object();
#ifdef TIMER_ON
    _timer.printTime("after tracking()");
#endif // TIMER_ON

    // publish status
    followme_laser::TrackingStatus msg;
    msg.status = _current_status;
    _status_pub.publish(msg);

    make_gui();

    ROS_INFO_THROTTLE(1, "Time for image_callback(): %g ms", _timer.getTimeMilliseconds());

  } // end image_callback();

  ////////////////////////////////////////////////////////////////////////////////

  /*!
   *\arg pt the point in the 2D image
   *Return the point in the camera frame
   */
  inline cv::Point3d project_to_3d (const cv::Point2i & pt) const {
    cv::Point3d line_vec = _cam_model.projectPixelTo3dRay(pt);
    cv::Point3d vec_bad_orien =
        line_vec // direction vector at pt
        *(_bridge_img_ptr->image.at<float>(pt.y, pt.x) // depth
          / geometry_utils::norm(line_vec) // direction vector norm at pt
          );
    //    ROS_INFO_THROTTLE(1, "(%i, %i) -> (%g, %g, %g)",
    //                      pt.x, pt.y, vec_bad_orien.x, vec_bad_orien.y, vec_bad_orien.z);
    //return vec_bad_orien;
    return cv::Point3d(vec_bad_orien.z, -vec_bad_orien.x, -vec_bad_orien.y);
    //return cv::Point3d(-vec_bad_orien.y, -vec_bad_orien.z, vec_bad_orien.x);
  } // end reproject()

  ////////////////////////////////////////////////////////////////////////////////


  /*!
   * \arg pt the point in the 3D image
   * \return the point in the camera frame
   *  Orientation:
   *   O-------------> x
   *   |
   * y V
   */
  inline cv::Point2d project_to_2d(const cv::Point3d & pt_cam_frame) const {
    //    ROS_INFO_THROTTLE(1, "project_to_2d('%s')",
    //                      geometry_utils::printP(pt_cam_frame).c_str());

    return _cam_model.project3dToPixel(pt_cam_frame);

    //    cv::Point2d pt_bad_orien = _cam_model.project3dToPixel(pt_cam_frame);
    //    return cv::Point2d(pt_bad_orien.y, pt_bad_orien.x);

    //    return _cam_model.project3dToPixel(cv::Point3d
    //                                       (pt_cam_frame.z, -pt_cam_frame.x,
    //                                        -pt_cam_frame.y));
  } // end reproject()

  //////////////////////////////////////////////////////////////////////////////

  inline cv::Point2d project_to_2d(const geometry_msgs::Point pt_cam_frame) const {
    return project_to_2d(cv::Point3d(pt_cam_frame.x, pt_cam_frame.y,
                                     pt_cam_frame.z));
  } // end reproject()

  //////////////////////////////////////////////////////////////////////////////

  //! set the new tracked object by its coordinate (x + y * cols)
  void tracked_object_coordinates_callback(const std_msgs::Int32ConstPtr& pt_msg) {
    ObjectName obj_name = pt_msg->data;
    // check if it is NO_OBJECT
    if (obj_name == fast_cluster_functions::NO_OBJECT) {
      stop_tracking_object();
      return;
    }

    // get the value at the clicked point
    _clicked_pt.x = obj_name % _components_illus.cols;
    _clicked_pt.y = obj_name / _components_illus.cols;
    _clicked_pt_timer.reset();
    start_tracking_new_object_at_pixel(_clicked_pt);
  } // end tracked_object_coordinates_callback();

  //////////////////////////////////////////////////////////////////////////////

  bool start_tracking_new_object_at_pixel(const cv::Point2i & clicked_pixel) {
    // get object index
    ObjectName clicked_object_name = fast_cluster_functions::NO_OBJECT;
    if (clicked_pixel.x < 0 || clicked_pixel.x >= (int) _cam_info->width
        || clicked_pixel.y < 0 || clicked_pixel.y >= (int) _cam_info->height) {
      ROS_INFO_THROTTLE
          (1, "The tracking seed at pixel (%i, %i) is outside of the image "
           "(%ix%i), not doing anything.",
           clicked_pixel.x, clicked_pixel.y, _cam_info->width, _cam_info->height);
      //stop_tracking_object();
      return false;
    }

    clicked_object_name = _objects_names.at<int>(clicked_pixel.y, clicked_pixel.x);
    ROS_INFO_THROTTLE(1, "The tracking seed at pixel (%i, %i) is object %i. ",
                      clicked_pixel.x, clicked_pixel.y, clicked_object_name);

    // set the new index
    return start_tracking_new_object(clicked_object_name);
  } // end start_tracking_new_object_at_pixel()

  //////////////////////////////////////////////////////////////////////////////

  bool start_tracking_new_object(const ObjectName & new_object_name) {
    ROS_INFO_THROTTLE(1, "start_tracking_new_object(object %i)",
                      new_object_name);
    // check if it is NO_OBJECT
    if (new_object_name == fast_cluster_functions::NO_OBJECT) {
      stop_tracking_object();
      return false;
    }
    _tracked_object_name = new_object_name;
    _current_status = followme_laser::TrackingStatus::TARGET_TRACKING_OK;
    _tracked_object_last_time_seen.reset();
    return true;
  } // end start_tracking_new_object();

  //////////////////////////////////////////////////////////////////////////////

  bool tracking_seed_callback
  (followme_laser::SetTrackingSeed::Request  & req,
   followme_laser::SetTrackingSeed::Response & res) {
    ROS_INFO_THROTTLE(1, "FastClusterDetector:tracking_seed_callback(%s)",
                      geometry_utils::pose_stamped_to_string(req.pose_in).c_str());
    // make some sanity checks
    if (req.pose_in.pose.orientation.x == 0 &&
        req.pose_in.pose.orientation.y == 0 &&
        req.pose_in.pose.orientation.z == 0 &&
        req.pose_in.pose.orientation.w == 0) {
      ROS_WARN("tracking_seed_callback: invalid seed %s, stopping tracking.",
               geometry_utils::pose_stamped_to_string(req.pose_in).c_str());
      stop_tracking_object();
      return false;
    }

    // convert to the camera frame
    geometry_msgs::PoseStamped pose__camera_frame = req.pose_in;
    if (req.pose_in.header.frame_id != _camera_frame) {
      //      ROS_INFO_THROTTLE(1, "TF Converting from '%s' to '%s'",
      //                        req.pose_in.header.frame_id.c_str(), _camera_frame.c_str());
      pose__camera_frame.header.frame_id = _camera_frame;
      try {
        _tf_listener.transformPose(_camera_frame, ros::Time(0),
                                   req.pose_in, _static_frame, pose__camera_frame);
      } catch (tf::ExtrapolationException e) {
        ROS_WARN("tracking_seed_callback:transform error:'%s', stopping tracking.",
                 e.what());
        stop_tracking_object();
        return false;
      }
      //ROS_INFO_THROTTLE(1, "TF conversion OK.");
    } // end if frames different

    // reproject point to image frame
    //  cv::Point2d foo = project_to_2d(cv::Point3d(0, 0, 0));
    //  ROS_WARN("project_to_2d(cv::Point3d(0, 0, 0)): (%g, %g)", foo.x, foo.y);
    //  foo = project_to_2d(cv::Point3d(1, 0, 0));
    //  ROS_WARN("project_to_2d(cv::Point3d(1, 0, 0)): (%g, %g)", foo.x, foo.y);
    //  foo = project_to_2d(cv::Point3d(0, 1, 0));
    //  ROS_WARN("project_to_2d(cv::Point3d(0, 1, 0)): (%g, %g)", foo.x, foo.y);
    //  foo = project_to_2d(cv::Point3d(0, 0, 1));
    //  ROS_WARN("project_to_2d(cv::Point3d(0, 0, 1)): (%g, %g)", foo.x, foo.y);
    cv::Point2d image_2d_pt = project_to_2d(pose__camera_frame.pose.position);
    _clicked_pt.x = image_2d_pt.x; _clicked_pt.y = image_2d_pt.y;
    _clicked_pt_timer.reset();

    // try to use that pixel
    bool success = start_tracking_new_object_at_pixel(_clicked_pt);
    if (!success)
      return false;

    // if success, check the distance from the chosen point to the reprojection
    // get the 3D point at that pixel
    cv::Point3d reprojected_image_pt = project_to_3d(_clicked_pt);
    cv::Point3d reprojected_image_pt_good_orien
        (-reprojected_image_pt.y, -reprojected_image_pt.z, reprojected_image_pt.x);

    double dist_pt_to_reproj = geometry_utils::distance_points3
        (pose__camera_frame.pose.position, reprojected_image_pt_good_orien);
    //  ROS_WARN("pose__camera_frame.pose.position:%s,\t _clicked_pt:%s,\t "
    //           "reprojected_image_pt:%s,\t dist_pt_to_reproj:%g",
    //           geometry_utils::printP(pose__camera_frame.pose.position).c_str(),
    //           geometry_utils::printP2(_clicked_pt).c_str(),
    //           geometry_utils::printP(reprojected_image_pt_good_orien).c_str(),
    //           dist_pt_to_reproj);

    double max_seed_reproj_distance = 1;
    if (dist_pt_to_reproj > max_seed_reproj_distance) { // meters
      ROS_WARN("tracking_seed_callback(): the distance between the wanted seed %s "
               "and its reprojection on the depth image %s is too high: "
               "%g > max=%g . Not tracking.",
               geometry_utils::printP(pose__camera_frame.pose.position).c_str(),
               geometry_utils::printP(reprojected_image_pt_good_orien).c_str(),
               dist_pt_to_reproj, max_seed_reproj_distance);
      stop_tracking_object();
      return false;
    }

    return true;
  } // end tracking_seed_callback();

  //////////////////////////////////////////////////////////////////////////////

  inline void stop_tracking_object() {
    ROS_INFO_THROTTLE(1, "stop_tracking_object()");
    _tracked_object_name = fast_cluster_functions::NO_OBJECT;
    _current_status = followme_laser::TrackingStatus::NO_TARGET;
    // sending a stop order to a possible tracker
    _stop_tracking_order_publisher.publish(std_msgs::Empty());
    // publish an empty pose to clean the pose in rviz
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = _static_frame;
    _tracked_object_pose_publisher.publish(msg);
  } // end stop_tracking_object();

  ///////////////////////////////////////////////////////////////////////////////

  /*! the function for tracking the object selected by the user.
      Reproject the wanted object and emit the point */
  inline void track_object() {
    if (_tracked_object_name == fast_cluster_functions::NO_OBJECT) {
      // clear marker
      _marker.action = 2; // delete
      _marker_pub.publish(_marker);
      return;
    } // end if (_tracked_object_name != fast_cluster_functions::NO_OBJECT)

    // try to get the points
    const ConnectedComponentsMatcher::Comp *pts, *pts_resized;
    const ConnectedComponentsMatcher::Bbox*bbox;
    bool lookup_success = matcher.reco_history_lookup
        (_tracked_object_name, 0, pts, pts_resized, bbox);
    if (!lookup_success) {
      if (_tracked_object_last_time_seen.getTimeMilliseconds() / 1000
          > tracking_timeout) {
        ROS_WARN("the tracked object %i has been lost for too long"
                 "(not found since %g ms, timeout:%f s), stopping.",
                 _tracked_object_name,
                 _tracked_object_last_time_seen.getTimeMilliseconds() / 1000.f,
                 tracking_timeout);
        _tracked_object_name = fast_cluster_functions::NO_OBJECT;
        _current_status = followme_laser::TrackingStatus::NO_TARGET;
        // clear marker
        _marker.action = 2; // delete
        _marker_pub.publish(_marker);
        return;
      }
      ROS_INFO_THROTTLE
          (1, "Impossible to find the tracked object %i "
           "(not found since %g ms, timeout:%f s)!",
           _tracked_object_name,
           _tracked_object_last_time_seen.getTimeMilliseconds() / 1000.f,
           tracking_timeout);
      _current_status = followme_laser::TrackingStatus::TARGET_TRACKING_LOST;
      // put a red marker (error)
      _marker.color.r = 1; _marker.color.g = 0; _marker.action = 0; // add
      _marker_pub.publish(_marker);
      return;
    }
    _tracked_object_last_time_seen.reset();
    // reset the tracked object center
    _tracked_object_pose_cam_frame.header.frame_id = _camera_frame;
    _tracked_object_pose_cam_frame.header.stamp = _img_msg_timestamp;
    _tracked_object_pose_cam_frame.pose.position.x = 0;
    _tracked_object_pose_cam_frame.pose.position.y = 0;
    _tracked_object_pose_cam_frame.pose.position.z = 0;
    _tracked_object_pose_cam_frame.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    // find the cluster average
    for (unsigned int curr_pt_idx = 0; curr_pt_idx < pts->size(); ++curr_pt_idx) {
      cv::Point3d curr_pt = project_to_3d((*pts)[curr_pt_idx]);
      _tracked_object_pose_cam_frame.pose.position.x += curr_pt.x;
      _tracked_object_pose_cam_frame.pose.position.y += curr_pt.y;
      _tracked_object_pose_cam_frame.pose.position.z += curr_pt.z;
    } // end loop curr_pt_idx
    _tracked_object_pose_cam_frame.pose.position.x *= 1.f / pts->size();
    _tracked_object_pose_cam_frame.pose.position.y *= 1.f / pts->size();
    _tracked_object_pose_cam_frame.pose.position.z *= 1.f / pts->size();

    // convert it into static frame
    try {
      //      _tf_listener.transformPose(_static_frame,  ros::Time(0),
      //                                 _tracked_object_pose_cam_frame,
      //                                 _static_frame,
      //                                 _tracked_object_pose_static_frame);
      _tf_listener.waitForTransform
          (_static_frame, _tracked_object_pose_cam_frame.header.frame_id,
           _img_msg_timestamp, ros::Duration(1));
      _tf_listener.transformPose
          (_static_frame, _tracked_object_pose_cam_frame,
           _tracked_object_pose_static_frame);
    } catch (tf::TransformException e) {
      ROS_WARN_THROTTLE(2, "transform error:'%s'", e.what());
      // put an orange marker (warning)
      _marker.color.r = 1; _marker.color.g = 0.5; _marker.action = 0; // add
      return;
    }

    ROS_DEBUG_THROTTLE(1, "The wanted object %i is in (%g, %g, %g) in frame '%s'"
                       "= (%g, %g, %g) in static frame '%s'.",
                       _tracked_object_name,
                       _tracked_object_pose_cam_frame.pose.position.x,
                       _tracked_object_pose_cam_frame.pose.position.y,
                       _tracked_object_pose_cam_frame.pose.position.z,
                       _tracked_object_pose_cam_frame.header.frame_id.c_str(),
                       _tracked_object_pose_static_frame.pose.position.x,
                       _tracked_object_pose_static_frame.pose.position.y,
                       _tracked_object_pose_static_frame.pose.position.z,
                       _static_frame.c_str());

    _tracked_object_pose_static_frame.header.stamp = _img_msg_timestamp;
    _tracked_object_pose_publisher.publish(_tracked_object_pose_static_frame);
    // green color (OK)
    _marker.color.r = 0; _marker.color.g = 1; _marker.action = 0; // add
    //    _marker.pose.position = _tracked_object_pose_cam_frame.pose.position;
    _marker.pose.position = _tracked_object_pose_static_frame.pose.position;
    _marker_pub.publish(_marker);
  } // end track_object();

  //////////////////////////////////////////////////////////////////////////////

  //! make a fancy interface
  inline void make_gui() {

    // fill _objects_names with the names of the recognized objects
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG
        || PUBLISH_OBJECTS_NAMES_IMG || PUBLISH_COMPONENTS_ILLUS) {
      _objects_names.create(edges_inverted.size());
      _objects_names = fast_cluster_functions::NO_OBJECT;
      for (unsigned int cluster_idx = 0; cluster_idx < _components_pts.size();
           ++cluster_idx) {
        ObjectName obj_name = 0;
        bool success = matcher.get_comp_name(0, cluster_idx, obj_name);
        if (!success)
          continue;
        DisjointSets2::Comp*curr_comp = &_components_pts.at(cluster_idx);
        for (unsigned int compt_pt_idx = 0; compt_pt_idx < curr_comp->size();
             ++compt_pt_idx) {
          _objects_names((*curr_comp)[compt_pt_idx]) = obj_name;
        } // end loop compt_pt_idx
      } // end loop cluster_idx
#ifdef TIMER_ON
      _timer.printTime("filling of _objects_names");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG
    //  || PUBLISH_OBJECTS_NAMES_IMG || PUBLISH_COMPONENTS_ILLUS


    // paint the matched components on _components_illus
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG || PUBLISH_COMPONENTS_ILLUS) {
      fast_cluster_functions::paint_object_image
          (_objects_names, _components_illus, _tracked_object_name);
      // put indices name on them
      for (unsigned int cluster_idx = 0; cluster_idx < _bounding_boxes.size();
           ++cluster_idx) {
        ObjectName obj_name = 0;
        bool success = matcher.get_comp_name(0, cluster_idx, obj_name);
        if (!success)
          continue;
        image_utils::draw_text_centered
            (_components_illus,
             StringUtils::cast_to_string(obj_name),
             .5 *(_bounding_boxes[cluster_idx].tl() + _bounding_boxes[cluster_idx].br()),
             CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 2);
      } // end loop cluster_idx

      // draw the clicked point
      if (_clicked_pt_timer.getTimeSeconds() < 2) {
        cv::circle(_components_illus, _clicked_pt, 6, CV_RGB(0, 255, 0), -1);
        cv::circle(_components_illus, _clicked_pt, 6, CV_RGB(0, 0, 0), 3);
      }
#ifdef TIMER_ON
      _timer.printTime("painting of _components_illus");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG || PUBLISH_COMPONENTS_ILLUS


    // make the collage on _img_gui
    if (SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG) {
      _img_gui.create(2 *img_uchar.rows, 3 *img_uchar.cols);
      _img_gui.setTo(0);
      // 1st row
      cv::cvtColor(img_uchar, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             0 *img_uchar.cols, 0 *img_uchar.rows, 0, "img_uchar");
      //#if _nan_removal_method == _VALUE_REMOVAL_METHOD_INPAINT
      //    cv::cvtColor(inpaint_mask, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      //    image_utils::paste_img(img_gui_rgb_buffer, _img_gui,
      //                           1 *img_uchar.cols, 0 *img_uchar.rows);
      //#endif // _VALUE_REMOVAL_METHOD_INPAINT
      cv::cvtColor(img_uchar_with_no_nan, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img
          (_img_gui_rgb_buffer, _img_gui,
           1 *img_uchar.cols, 0 *img_uchar.rows, 0,
           std::string("img_uchar_with_no_nan: m") +
           StringUtils::cast_to_string(_nan_removal_method));
      cv::cvtColor(edges_inverted, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             2 *img_uchar.cols, 0 *img_uchar.rows, 0, "edges_inverted");

      // 2nd row
      cv::cvtColor(edges_inverted_opened, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             0 *img_uchar.cols, 1 *img_uchar.rows, 0,
                             "edges_inverted_opened");
      cv::cvtColor(edges_inverted_opened_with_nan, _img_gui_rgb_buffer, cv::COLOR_GRAY2RGB);
      image_utils::paste_img(_img_gui_rgb_buffer, _img_gui,
                             1 *img_uchar.cols, 1 *img_uchar.rows, 0,
                             "edges_inverted_opened_with_nan");
      image_utils::paste_img(_components_illus, _img_gui,
                             2 *img_uchar.cols, 1 *img_uchar.rows, 0, "components_illus");
#ifdef TIMER_ON
      _timer.printTime("collage in _img_gui");
#endif // TIMER_ON
    } // end if SAVE_IMAGES || DISPLAY || PUBLISH_GUI_IMG


    if (SAVE_IMAGES) {
      cv::imwrite("0-img_gui.png", _img_gui);
      cv::imwrite("1-img_uchar.png", img_uchar);
      cv::imwrite("2-img_uchar_with_no_nan.png", img_uchar_with_no_nan);
      cv::imwrite("3-edges_inverted.png", edges_inverted);
      cv::imwrite("4-edges_inverted_opened.png", edges_inverted_opened);
      cv::imwrite("5-edges_inverted_opened_with_nan.png", edges_inverted_opened_with_nan);
      cv::imwrite("6-components_illus.png", _components_illus);
#ifdef TIMER_ON
      _timer.printTime("image saving");
#endif // TIMER_ON
    } // end if (SAVE_IMAGES)


    if (PUBLISH_GUI_IMG) {
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = _static_frame;
      out_msg.header.stamp    = _img_msg_timestamp;
      out_msg.encoding        = sensor_msgs::image_encodings::BGR8;
      out_msg.image           = _img_gui;
      _objects_img_gui_pub.publish(out_msg.toImageMsg());
#ifdef TIMER_ON
      _timer.printTime("_img_gui publishing");
#endif // TIMER_ON
    } // end if PUBLISH_GUI_IMG


    if (PUBLISH_COMPONENTS_ILLUS) {
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = _static_frame;
      out_msg.header.stamp    = _img_msg_timestamp;
      out_msg.encoding        = sensor_msgs::image_encodings::BGR8;
      out_msg.image           = _components_illus;
      _components_illus_pub.publish(out_msg.toImageMsg());
#ifdef TIMER_ON
      _timer.printTime("_objects_names publishing");
#endif // TIMER_ON
    } // end if (PUBLISH_COMPONENTS_ILLUS)

    if (PUBLISH_OBJECTS_NAMES_IMG) {
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = _static_frame;
      out_msg.header.stamp    = _img_msg_timestamp;
      out_msg.encoding        = sensor_msgs::image_encodings::TYPE_32SC1;
      out_msg.image           = _objects_names;
      _objects_names_img_pub.publish(out_msg.toImageMsg());
#ifdef TIMER_ON
      _timer.printTime("_objects_names publishing");
#endif // TIMER_ON
    } // end if (PUBLISH_OBJECTS_NAMES_IMG)

    // publish the number of objects
    std_msgs::Int32 objects_nb_pub_msg;
    objects_nb_pub_msg.data = matcher.get_number_of_recognized_objects();
    _objects_nb_pub.publish(objects_nb_pub_msg);
    //ROS_INFO("Publishing _objects_nb:%i", objects_nb_pub_msg.data);

    if (DISPLAY) {
      cv::imshow("img_gui", _img_gui);
      char key_code = cv::waitKey(1);
      if ((int) key_code == 27)
        exit(0);
      // change the NaN handling
      else if (key_code == 'n') {
        _nan_removal_method = (NaNRemovalMethod) ((_nan_removal_method + 1) % 3);
        ROS_WARN("Changing _nan_removal_method: now %i", _nan_removal_method);
      }
#ifdef TIMER_ON
      _timer.printTime("after imshow()");
#endif // TIMER_ON
    } // end if (DISPLAY)

  } // end make_gui();

  //////////////////////////////////////////////////////////////////////////////


private:
  /**
  *NodeHandle is the main access point to communications with the ROS system.
  *The first NodeHandle constructed will fully initialize this node, and the last
  *NodeHandle destructed will close down the node.
  */
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;

  // status stuff
  int _current_status;
  ros::Publisher _status_pub;

  ros::ServiceServer _tracking_seed_srv;
  // float receiver
  Timer _timer;
  ros::Time _img_msg_timestamp; //!< when the message was received
  cv_bridge::CvImageConstPtr _bridge_img_ptr;
  image_transport::Subscriber _depth_image_subscriber;

  // float -> uchar
  double alpha_trans;
  double beta_trans;
  cv::Mat1b img_uchar;
  cv::Mat src_float_clean_buffer;

  // nan removal
  NaNRemovalMethod _nan_removal_method;
  // VALUE_REMOVAL_METHOD_INPAINT
  cv::Mat1b inpaint_mask;
  // VALUE_REMOVAL_METHOD_AVERAGE_BORDER
  cv::Rect roi;
  cv::Mat1b img_uchar_with_no_nan;

  // edge detection
  double canny_param1, canny_param2;
  int canny_tb1_value, canny_tb2_value;

  cv::Mat1b edges;
  cv::Mat1b edges_inverted;
  cv::Mat1b edges_inverted_opened_with_nan;
  cv::Mat1b edges_inverted_opened;
  //cv::Mat1b harrisCorners;

  // connected comps
  DisjointSets2 _set;
  std::vector<std::vector<cv::Point> > _components_pts;
  std::vector<cv::Rect> _bounding_boxes;
  cv::Mat1i _objects_names; //!< will be filled with object names
  cv::Mat3b _components_illus;
  //! the point clicked by the user on the GUI
  cv::Point _clicked_pt;
  Timer _clicked_pt_timer;

  // clustering
  image_transport::Publisher _objects_names_img_pub;
  image_transport::Publisher _components_illus_pub;
  ConnectedComponentsMatcher matcher;
  ros::Publisher _objects_nb_pub;

  // tracking part
  ros::Subscriber _tracked_object_coordinates_subscriber;
  ObjectName _tracked_object_name;
  ros::Publisher _tracked_object_pose_publisher;
  ros::Publisher _stop_tracking_order_publisher;
  Timer _tracked_object_last_time_seen;

  /*reprojection and filtering */
  sensor_msgs::CameraInfoConstPtr _cam_info;
  image_geometry::PinholeCameraModel _cam_model;
  tf::TransformListener _tf_listener;

  // results
  std::string _camera_frame;
  geometry_msgs::PoseStamped _tracked_object_pose_cam_frame;
  geometry_msgs::PoseStamped _tracked_object_pose_static_frame;
  visualization_msgs::Marker _marker;
  ros::Publisher _marker_pub;

  // GUI
  cv::Mat3b _img_gui;
  cv::Mat3b _img_gui_rgb_buffer;
  image_transport::Publisher _objects_img_gui_pub;
}; // end class FastClusterDetector

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char**argv) {
  ros::init(argc, argv, "FastClusterDetector");
  FastClusterDetector detec;
  ros::spin();
  return 0;
}
