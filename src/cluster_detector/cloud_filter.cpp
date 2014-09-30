/*!
  \file        cloud_filter.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2011

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

A simple node for compressing point clouds.


\section Parameters
  - \b "input_topic"
        [string] (default: "camera/depth/points")
        Where to get the uncompressed point clouds.

  - \b "output_topic"
        [string] (default: "camera/depth/points_compressed")
        Where to publish the uncompressed point clouds.

  - \b "output_frame"
        [string] (default: "/base_link")
        The wanted TF frame for the output topic.
        If different from the input frame, a TF conversion is done.

\section Subscriptions
  - \b  {input_topic}
        [pcl::PointCloud<pcl::PointXYZ>]
        The raw, uncompressed point cloud

\section Publications
  - \b  {output_topic}
        [pcl::PointCloud<pcl::PointXYZ>]
        The compressed point cloud

 */

#include "debug/error.h"
#include "time/timer.h"

// ROS
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>
// PCL
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/impl>
#include <pcl/segmentation/extract_clusters.h>
// Need to include the pcl ros utilities
//#include <pcl/point_cloud.h>
//#include <pcl/common/transforms.h>
// pcl conversions to indigo
#include <pcl_conversions/pcl_conversions.h>

namespace multi_modal {

class CloudFilter : public nodelet::Nodelet {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

  //! the size of the leaves for the downsampling (meters)
  static const double LEAF_SIZE = 0.1; // 0.05;

  //! the max rate for the filter (Hz)
  static const int MAX_RATE = 3;

  virtual void onInit() {
    NODELET_INFO("onInit()");
    transform_listener = new tf::TransformListener(ros::Duration(10, 0));
    src_to_dst_transform_found= false;

    // get params
    ros::NodeHandle & nh_public = getNodeHandle(), & nh_private = getPrivateNodeHandle();
    std::string input_topic = "camera/depth/points";
    std::string output_topic = "camera/depth/points_filtered";
    output_frame = "/base_link";
    nh_private.param("input_topic", input_topic, input_topic);
    nh_private.param("output_topic", output_topic, output_topic);
    nh_private.param("output_frame", output_frame, output_frame);

    // subscribers
    _cloud_subscriber = nh_public.subscribe<Cloud>
        (input_topic, 1, &CloudFilter::filter_cloud, this);

    // publishers
    _cloud_publisher = nh_public.advertise<Cloud>(output_topic, 1);

    _cloud_filtered_src_frame = new Cloud();

    ROS_WARN("CloudFilter: getting clouds on '%s', publishing the filtered on '%s'"
              ", output_frame:'%s'",
             _cloud_subscriber.getTopic().c_str(),
             _cloud_publisher.getTopic().c_str(),
             output_frame.c_str());
    //_cloud_filtered_ptr = Cloud::Ptr(_cloud_filtered_src_frame);
  } // end onInit();

  ////////////////////////////////////////////////////////////////////////////

  ~CloudFilter() {
    delete transform_listener;
  } // end dtor

  ////////////////////////////////////////////////////////////////////////////

  void filter_cloud(const Cloud::ConstPtr& cloud_msg) {
    ROS_INFO_THROTTLE(1, "filter_cloud()");
    if (_cloud_publisher.getNumSubscribers() == 0) {
      ROS_INFO_THROTTLE(1, "CloudFilter: no subscribers to '%s', not filtering cloud.",
                        _cloud_publisher.getTopic().c_str());
      return;
    }

    Timer timer;
    ros::Rate rate(MAX_RATE);

    // Create the filtering object: downsample the dataset
    // using a custom leaf size
    timer.reset();
    _cloud_filtered_src_frame->points.clear();
    _vg.setInputCloud (cloud_msg);
    _vg.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    _vg.filter (*_cloud_filtered_src_frame);
    Timer::Time time_filtering = timer.getTimeMilliseconds();
    //    maggiePrint("orig_stamp:%g, diff with now:%g",
    //                _cloud_filtered->header.stamp.toSec(),
    //                (ros::Time::now() - _cloud_filtered->header.stamp).toSec() );
    //    _cloud_filtered->header.stamp = ros::Time::now() - ros::Duration(3);
    //    maggiePrint("orig_stamp:%g, diff with now:%g",
    //                _cloud_filtered->header.stamp.toSec(),
    //                (ros::Time::now() - _cloud_filtered->header.stamp).toSec() );

    // convert the referential
    timer.reset();

    std::string src_frame = _cloud_filtered_src_frame->header.frame_id;
    if (!src_to_dst_transform_found) {
      maggieDebug2("Waiting for the tf '%s'->'%s'",
                   src_frame.c_str(), output_frame.c_str());
      std::string error_msg;
      bool tf_received =
          transform_listener->waitForTransform
          (output_frame, src_frame, ros::Time::now(), ros::Duration(5.f),
           ros::Duration(0.01), &error_msg);
      if (tf_received) {
        maggieDebug1("tf '%s'->'%s' waited and successfully received.",
                     src_frame.c_str(), output_frame.c_str());
        transform_listener->lookupTransform(output_frame, src_frame,
                                           ros::Time(),
                                           src_to_dst_transform);
        src_to_dst_transform_found = true;
      }
      else {
        maggieDebug1("tf '%s'->'%s' waited, returned error '%s'. Aborting.",
                     src_frame.c_str(), output_frame.c_str(), error_msg.c_str());
        return;
      } // end if tf_received
    } // end if not src_to_dst_transform_found

    //    std::string src_frame = _cloud_filtered->header.frame_id;
    //     // check if we can convert
    //    if (!transform_listener->canTransform
    //        (output_frame, src_frame, ros::Time::now())) {
    //      maggieDebug2("Cannot convert '%s'->'%s'",
    //                   src_frame.c_str(), output_frame.c_str());
    //      return;
    //      maggieDebug2("Waiting for the tf '%s'->'%s'",
    //                   src_frame.c_str(), output_frame.c_str());
    //      std::string error_msg;
    //      bool tf_received =
    //          transform_listener->waitForTransform
    //          (output_frame, src_frame, ros::Time::now(), ros::Duration(1.f),
    //           ros::Duration(0.01), &error_msg);
    //      if (tf_received) {
    //        maggieDebug1("tf '%s'->'%s' waited and successfully received.",
    //                     src_frame.c_str(), output_frame.c_str());
    //      }
    //      else {
    //        maggieDebug1("tf '%s'->'%s' waited, returned error '%s'. Aborting.",
    //                     src_frame.c_str(), output_frame.c_str(), error_msg.c_str());
    //        return;
    //      } // end if tf_received
    //    } // end if !canTransform()

    // make the proper conversion
    //    bool conv_success = pcl_ros::transformPointCloud
    //        (output_frame,
    //         *_cloud_filtered, *_cloud_filtered,
    //         transform_listener);
    //    if (!conv_success) {
    //      maggieDebug2("Error while transformPointCloud(). Aborting.");
    //    }
    _cloud_filtered_output_frame.clear();
    _cloud_filtered_output_frame.header.frame_id = output_frame;
    pcl::transformPointCloud(*_cloud_filtered_src_frame, _cloud_filtered_output_frame,
                                 src_to_dst_transform);

    maggieDebug2("Filtering: %g ms - Converting: %g ms, "
                 "\tcloud before filtering has %i pts, "
                 "after filtering %i.",
                 time_filtering,
                 timer.getTimeMilliseconds(),
                 cloud_msg->points.size(),
                 _cloud_filtered_src_frame->points.size());

    // emit it
    if (_cloud_publisher.getNumSubscribers() > 0)
      _cloud_publisher.publish(*_cloud_filtered_src_frame);

    // wait
    rate.sleep();
  } // end filter_cloud()

  //////////////////////////////////////////////////////////////////////////////

private:
  std::string output_frame;

  // filtering
  pcl::VoxelGrid<pcl::PointXYZ> _vg;
  Cloud* _cloud_filtered_src_frame;
  Cloud _cloud_filtered_output_frame;
  //Cloud::Ptr _cloud_filtered_ptr;
  ros::Subscriber _cloud_subscriber;
  ros::Publisher _cloud_publisher;

  // transformer
  tf::TransformListener* transform_listener;
  tf::StampedTransform src_to_dst_transform;
  bool src_to_dst_transform_found;
}; // end class CloudFilter

} // end namespace multi_modal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(multi_modal, CloudFilter, multi_modal::CloudFilter, nodelet::Nodelet)
