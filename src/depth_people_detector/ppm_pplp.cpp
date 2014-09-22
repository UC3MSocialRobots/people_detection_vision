/*!
  \file        ppm_pplp.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/7/10

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

Tests for \file ppm_pplp.h
rosparam set /kinect_only/kinect_serial_number A00365A10630110A
ROS_NAMESPACE=kinect_only rosrun people_msgs ppm_pplp.exe --activate

 */
#if 0
#include "ppm_pplp.h"
#include "skill_templates/ros_vision_skill.h"

class PpmSkill: public VisionSkill {
public:
  //! constructor
  PpmSkill(unsigned long time_cycle_us,
           RobotConfig* robot_config,
           const bool DISPLAY_ = false) :
    VisionSkill(time_cycle_us,
                "MOVEMENTLISTENER_START", "MOVEMENTLISTENER_STOP",
                robot_config,
                DISPLAY_)
  {
    _data_acquisition_want_rgb = true;
    _data_acquisition_want_depth = true;
    _data_acquisition_by_wait_for_message = false;
  }

  //////////////////////////////////////////////////////////////////////////////

  void custom_launch(void) {
  } // end custom_launch();

  /////////////////////////////////////////////////////////////////////////////

  void custom_end(void) {
  }

  //////////////////////////////////////////////////////////////////////////////

  void update(void) {
    rois(frame(), frame_depth(), depth_camera_model,
         ppm, ppm_thres, set, comps_points, comps_images);

    cv::imshow("bgr", frame());
    //cv::imshow("depth", image_utils::depth2viz(frame_depth(), image_utils::FULL_RGB_STRETCHED));
    ppm.convertTo(ppm_float, CV_32FC1);
    if (!ppm_float.empty())
      cv::imshow("ppm_float", image_utils::depth2viz(ppm_float, image_utils::FULL_RGB_STRETCHED, 3));
    //cv::imshow("ppm_thres", ppm_thres);
    image_utils::paste_images(comps_images, comps_images_collage, true, 200, 150);
    cv::imshow("comps_images_collage", comps_images_collage);
    cv::waitKey(50);
  }

protected:
  Ppm ppm;
  DisjointSets2 set;
  cv::Mat1b ppm_thres;
  std::vector<std::vector<cv::Point3f> > comps_points;
  std::vector<cv::Mat > comps_images;

  cv::Mat1f ppm_float;
  cv::Mat comps_images_collage;
}; // end class PpmSkill

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "PpmSkill");
  PpmSkill::launcher_simple<PpmSkill>(argc, argv, 50 * 1000);
}

#endif

#include "ppm_pplp.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "Ppm");
  Ppm skill;
  ros::spin();
  return 0;
}
