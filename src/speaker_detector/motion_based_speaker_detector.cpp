/*!
  \file        motion_based_speaker_detector.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/1/23

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

\class  MotionBasedSpeakerDetectorSkill

\section Parameters
  - \b "~ppl_input_topic"
        [string] (default: "ppl")
        Where the face recognition results will be obtained.

\section Subscriptions
  - \b ${ppl_input_topic}
        [people_msgs::PeoplePoseList]
        The found faces ROIs and the name of the persons recognized

 */
//ROS
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
// vision
#include <people_msgs/PeoplePoseList.h>
#include <vision_utils/skill_templates/nano_skill.h>
#include "motion_based_speaker_detector.h"
// people_msgs
#include <src/ppl_utils/ppl_attributes.h>

class MotionBasedSpeakerDetectorSkill : public NanoSkill {
public:
  MotionBasedSpeakerDetectorSkill()
    : NanoSkill("MOTION_BASED_SPEAKER_DETECTOR_START",
                "MOTION_BASED_SPEAKER_DETECTOR_STOP") {
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    maggieDebug2("create_subscribers_and_publishers()");

    // start face detection
    _face_detector_start_pub = _nh_public.advertise<std_msgs::Int16>
        ("FACE_DETECTOR_PPLP_START", 1);
    _face_detector_start_pub.publish(std_msgs::Int16());

    // get the topic names
    ros::NodeHandle nh_public, nh_private("~");
    std::string ppl_input_topic = "ppl";
    nh_private.param("ppl_input_topic",
                     ppl_input_topic,
                     ppl_input_topic);
    // susbscribe to face detection results
    _ppl_sub = nh_public.subscribe
                            (ppl_input_topic, 1,
                             &MotionBasedSpeakerDetectorSkill::ppl_cb, this);

    // advertize for the updated PPL
    _ppl_pub = nh_private.advertise
                            <people_msgs::PeoplePoseList>("ppl", 1);
  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  void ppl_cb(const people_msgs::PeoplePoseListConstPtr & msg) {
    unsigned int nposes = msg->poses.size();
    ROS_INFO_THROTTLE(1, "ppl_cb(nposes):%i", nposes);
    std::vector<unsigned int> assign_face2pose;
    // keep all new faces
    curr_faces.clear();
    curr_rois.clear();
    cv_bridge::CvImageConstPtr img_ptr;
    boost::shared_ptr<void const> tracked_object;
    for (unsigned int pose_idx = 0; pose_idx < nposes; ++pose_idx) {
      const people_msgs::PeoplePose* curr_pose = &(msg->poses[pose_idx]);
      if (curr_pose->rgb.width == 0 || curr_pose->rgb.height == 0) // if the pose has no image, cant compute activity
        continue;
      try { // decompress image
        img_ptr = cv_bridge::toCvShare(curr_pose->rgb, tracked_object,
                                       sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception e) {
        ROS_WARN("cv_bridge exception:'%s'", e.what());
        continue;
      }
      // keep correspondance
      assign_face2pose.push_back(pose_idx);

      // add decompressed image
      curr_faces.push_back(img_ptr->image.clone());
      cv::Rect face_ROI;
      face_ROI.x = curr_pose->images_offsetx;
      face_ROI.y = curr_pose->images_offsety;
      face_ROI.width = curr_pose->rgb.width;
      face_ROI.height = curr_pose->rgb.height;
      curr_rois.push_back(face_ROI);
    } // end loop pose_idx

    // compute non_verbal_activity() on these faces
    std::vector<NonVerbalActivity> activities;
    non_verbal_activity(prev_faces, prev_rois, curr_faces, curr_rois, activities, buffer);
    ROS_INFO_THROTTLE(1, "activities:%s",
                      StringUtils::accessible_to_string(activities).c_str());

    // publish new message
    people_msgs::PeoplePoseList msg_with_activity = *msg;
    for (unsigned int act_idx = 0; act_idx < activities.size(); ++act_idx) {
      unsigned int pose_idx = assign_face2pose[act_idx];
      ppl_utils::set_attribute(msg_with_activity.poses[pose_idx],
                                            "non_verbal_activity",
                                            activities[act_idx]);
    } // end loop act_idx
    _ppl_pub.publish(msg_with_activity);

    // store data
    prev_rois = curr_rois;
    prev_faces.clear();
    for (unsigned int face_idx = 0; face_idx < curr_faces.size(); ++face_idx)
      prev_faces.push_back(curr_faces[face_idx].clone());
  } // end ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers()  {
    maggieDebug2("shutdown_subscribers_and_publishers()");
    _face_detector_start_pub.shutdown();
    _ppl_sub.shutdown();
  } // end shutdown_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

private:

  ros::Publisher _face_detector_start_pub;
  //! face detection sub
  ros::Subscriber _ppl_sub;
  //! PPL pub
  ros::Publisher _ppl_pub;
  std::vector<cv::Mat> curr_faces, prev_faces;
  std::vector<cv::Rect> curr_rois, prev_rois;
  cv::Mat buffer;
}; // end class FaceCount

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "MotionBasedSpeakerDetectorSkill");
  MotionBasedSpeakerDetectorSkill skill;
  ros::spin();
  return 0;
}
