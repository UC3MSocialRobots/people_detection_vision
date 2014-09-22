/*!
  \file        nite_user_mask_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/27

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

A node that subscribes to the three data streams published by
the NITE middleware:
* RGB stream,
* depth stream,
* user mask stream,
and converts them into a people_msgs::PeoplePoseList message
that contains both the 3D position of the users and
their color and depth masks.

It is computationnaly very light, as the only computation needed
is the determination of the center of masses of each value
in the user mask.

\section Parameters
  - \b ~user_mask_topic
        [string] (default: "user")
        Where to get the user mask images.

  - \b ~depth_topic
        [std::string] (default: "depth")
        where to get the depth images

\section Subscriptions
  - \b {user_mask_topic}, {depth_topic}
        [sensor_msgs::Image]
        The user mask and depth image

\section Publications
  - \b "~ppl"
        [people_msgs::PeoplePoseList]
        The detected users in the mask

 */

#ifndef NITE_USER_MASK_PPLP_H
#define NITE_USER_MASK_PPLP_H

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
// #define USE_EXACT_TIME // comment to use approx time
#ifdef USE_EXACT_TIME
#include <message_filters/sync_policies/exact_time.h>
#else // not USE_EXACT_TIME
#include <message_filters/sync_policies/approximate_time.h>
#endif

// AD
#include <src/map/map_utils.h>
#include <src/time/timer.h>
#include <src/templates/pplp_template.h>
// people_msgs
#include <src/ppl_utils/rgb_depth_user2ppl.h>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_WARN(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

class NiteUserMask2Ppl : public PPLPublisherTemplate {
public:
  typedef sensor_msgs::Image Image;
  // use of message filters - inspiration available at
  // http://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
  typedef image_transport::SubscriberFilter ImgSub;
  static const unsigned int QUEUE_SIZE = 3;

  NiteUserMask2Ppl()
    : PPLPublisherTemplate("NITE_USER_MASK_PPLP_START", "NITE_USER_MASK_PPLP_STOP"),
      _it(_nh_public) {
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  //! should be extended for any kind of display using cv::imshow(), etc.
  virtual void display(const cv::Mat3b & rgb,
                       const cv::Mat1f & depth,
                       const cv::Mat1b & user_mask) {
    user_image_to_rgb(user_mask, user_illus);
    std::vector<cv::Point> coms2D;
    map_utils::map_values_to_container(_ppl_conv.get_coms(), coms2D);
    for (unsigned int com2D_idx = 0; com2D_idx < coms2D.size(); ++com2D_idx)
      cv::circle(user_illus, coms2D[com2D_idx], 5, CV_RGB(255,255,255), -1);
    cv::imshow("NiteUserMask2Ppl", user_illus);
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // get the image channel
    std::string _rgb_topic = "rgb";
    _nh_private.param("rgb_topic", _rgb_topic, _rgb_topic);
    _resolved_rgb_topic = _nh_public.resolveName(_rgb_topic);
    std::string _depth_topic = "depth";
    _nh_private.param("depth_topic", _depth_topic, _depth_topic);
    _resolved_depth_topic = _nh_public.resolveName(_depth_topic);
    std::string _user_topic = "user";
    _nh_private.param("user_topic", _user_topic, _user_topic);
    _resolved_user_topic = _nh_public.resolveName(_user_topic);

    // subscribe to the image message
    _rgb_sub = new ImgSub(_it, _resolved_rgb_topic, 1);
    _depth_sub = new ImgSub(_it, _resolved_depth_topic, 1);
    _user_sub = new ImgSub(_it, _resolved_user_topic, 1);

    // ApproximateTime synchronizer
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    _sync =  new message_filters::Synchronizer<MySyncPolicy>
             (MySyncPolicy(QUEUE_SIZE), *_rgb_sub, *_depth_sub, *_user_sub);
    _sync->registerCallback(boost::bind(&NiteUserMask2Ppl::rgb_depth_user_cb, this, _1, _2, _3));

    printf("NiteUserMask2Ppl: subscribing to '%s', '%s', '%s', "
           "publishing PPL to '%s'\n",
           _resolved_rgb_topic.c_str(),
           _resolved_depth_topic.c_str(),
           _resolved_user_topic.c_str(),
           get_ppl_topic().c_str());
  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    DEBUG_PRINT("shutdown_subscribers_and_publishers()\n");
    _rgb_sub->unsubscribe();
    _depth_sub->unsubscribe();
    _user_sub->unsubscribe();
    delete _rgb_sub;
    delete _depth_sub;
    delete _user_sub;
    // delete _sync; // also delete the subscribers it includes
  }

  //////////////////////////////////////////////////////////////////////////////

  void rgb_depth_user_cb(const Image::ConstPtr& rgb_msg,
                         const Image::ConstPtr& depth_msg,
                         const Image::ConstPtr& user_msg) {
    DEBUG_PRINT("rgb_depth_user_cb(%i, %i)\n", rgb_msg->width, rgb_msg->height);
    try {
      _rgb_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
      _depth_bridge = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      _user_bridge = cv_bridge::toCvShare(user_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      _images_header = rgb_msg->header;
    } catch (cv_bridge::Exception& e) {
      printf("/!\\ cv_bridge exception: %s\n", e.what());
      return;
    }
    //  image_utils::imwrite_debug("rgb.png", _rgb_bridge->image);
    //  image_utils::imwrite_debug("depth.png", _depth_bridge->image);
    //  image_utils::imwrite_debug("user.png", _user_bridge->image);
    //  cv::imshow("rgb", _rgb_bridge->image);
    //  cv::imshow("depth", _depth_bridge->image);
    //  cv::imshow("user", _user_bridge->image);
    //  cv::waitKey(0);
    process_rgb_depth_user(_rgb_bridge->image,
                           _depth_bridge->image,
                           _user_bridge->image);
  } // end rgb_depth_user_cb();

  //////////////////////////////////////////////////////////////////////////////

  inline void process_rgb_depth_user(const cv::Mat3b& rgb,
                                     const cv::Mat1f& depth,
                                     const cv::Mat1b& user_mask) {
    Timer timer;
    // compte center of masses
    if (!_ppl_conv.convert(rgb, depth, user_mask, NULL, &_images_header)) {
      return;
    }
    _ppl_conv.get_ppl().method = "nite_user_mask_pplp";
    publish_PPL(_ppl_conv.get_ppl());
    //  printf("NiteUserMask2Ppl::process(): %g ms, publishing a PPL of %i people\n",
    //                    timer.time(), _ppl_conv.nusers());
  } // end process();

  //////////////////////////////////////////////////////////////////////////////

protected:
  std_msgs::Header _images_header; //!< the header of the last received frame
  cv::Mat3b user_illus;

  //! where the camera frames are obtained
  std::string _resolved_rgb_topic, _resolved_depth_topic, _resolved_user_topic;

#ifdef USE_EXACT_TIME
  typedef message_filters::sync_policies::ExactTime<Image, Image, Image> MySyncPolicy;
#else // not USE_EXACT_T*IME
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;
#endif
  image_transport::ImageTransport _it;
  ImgSub *_rgb_sub, *_depth_sub, *_user_sub;
  message_filters::Synchronizer<MySyncPolicy>* _sync;
  cv_bridge::CvImage::ConstPtr _rgb_bridge, _depth_bridge, _user_bridge;
  //! information sharing
  ppl_utils::RgbDepthUser2PPL _ppl_conv;
}; // end NiteUserMask2Ppl

#endif // NITE_USER_MASK_PPLP_H
