/*!
  \file        rlpd_grabber.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11

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

Parameters:
  - \b "~rgb_topic", "~depth_topic", "~user_topic"
        [std::string, default "rgb", "depth", "user"]
        The name of the topic where the RGB image, depth image,
        user image streams can be obtained.

  - \b "~display"
      [bool, default: false]
      True to display the acquired RGB, depth, user images
 */
// #define USE_BACKGROUND_SUBSTRACTION
#ifdef USE_BACKGROUND_SUBSTRACTION
#include "people_detection_vision/bg_substractor_pplp.h"
#include "kinect/nite_subscriber_template.h"


class RLPDGrabber : public NiteSubscriberTemplate, public BackgroundSubstractorPPLP
{
public:
  virtual void init() {
    NiteSubscriberTemplate::init();
  }

  void fn(const cv::Mat3b & rgb,
          const cv::Mat1f & depth,
          const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list) {
    ROS_INFO_THROTTLE(1, "RLPDGrabber:fn()");
    // pass header from NiteSubscriberTemplate to BackgroundSubstractorPPLP
    _images_header = _last_header;
    BackgroundSubstractorPPLP::process_rgb_depth(rgb, depth);
    //BackgroundSubstractorPPLP::display(rgb, depth);
  } // end image_callback();

private:
  std_msgs::Header _images_header; //!< the header of the last received frame
}; // end class RLPDGrabber

////////////////////////////////////////////////////////////////////////////////
#else // no USE_BACKGROUND_SUBSTRACTION
////////////////////////////////////////////////////////////////////////////////
#include "vision_utils/rgb_depth_user_skill.h"


class RLPDGrabber : public RgbDepthUserSkill {
public:
  RLPDGrabber() : RgbDepthUserSkill("RLPD_GRABBER_START", "RLPD_GRABBER_STOP") {
    _was_beginning_stamp_defined = false;
  }

  virtual void process_rgb_depth_user(const cv::Mat3b & rgb,
                                      const cv::Mat1f & depth,
                                      const cv::Mat1b & user) {
    ROS_INFO_THROTTLE(1, "process_rgb_depth_user()");
    if (!_was_beginning_stamp_defined) {
      _was_beginning_stamp_defined = true;
      _beginning_stamp = _images_header.stamp;
    }
    std::ostringstream prefix;
    int msec = (_images_header.stamp - _beginning_stamp).toSec() * 1000;
    prefix << "/tmp/" << std::setw(6) << std::setfill('0') << msec;
    if (!vision_utils::write_rgb_depth_user_to_image_file
        (prefix.str(), &rgb, &depth, &user, &user_illus)) {
      printf("Could not save the images to '%s'!\n", prefix.str().c_str());
    }
  } // end image_callback();

  virtual void create_subscribers_and_publishers()   {}
  virtual void shutdown_subscribers_and_publishers() {}

private:
  bool _was_beginning_stamp_defined;
  ros::Time _beginning_stamp;
  cv::Mat3b user_illus;
}; // end class RLPDGrabber

#endif // USE_BACKGROUND_SUBSTRACTION
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rlpd_grabber");
  RLPDGrabber receiver;
#ifdef USE_BACKGROUND_SUBSTRACTION
  receiver.init();
#else
  receiver.start();
#endif // USE_BACKGROUND_SUBSTRACTION
  ros::spin();
}
