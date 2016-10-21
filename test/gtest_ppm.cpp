/*!
  \file        gtest_ppm.cpp
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

 */
#include <vision_utils/utils/rosmaster_alive.h>
#include "vision_utils/pplp_testing.h"
// people_msgs_rl
#include "vision_utils/test_person_histogram_set_variables.h"
#include "people_detection_vision/ppm_pplp.h"
// opencv
#include <opencv2/highgui/highgui.hpp>

namespace testvar = test_person_histogram_set_variables;

//#define DISPLAY

////////////////////////////////////////////////////////////////////////////////
#if 1
void direct_test(const std::string & filename_prefix) {
  if (!rosmaster_alive()) return;
  printf("\ndirect_test('%s')\n", filename_prefix.c_str());
  std::string kinect_serial_number = DEFAULT_KINECT_SERIAL();
  // read images
  cv::Mat bgr, depth;
  image_utils::read_rgb_and_depth_image_from_image_file(filename_prefix, &bgr, &depth);
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  kinect_openni_utils::read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);

  // call
  Ppm ppm;
  std::vector<std::vector<cv::Point3f> > comps_points;
  std::vector<cv::Mat3b> comps_images;
  std::vector<geometry_utils::Rect3f> comps_bboxes;
  ppm.rois(bgr, depth, depth_camera_model, comps_points, comps_images, comps_bboxes);
  ASSERT_TRUE(comps_images.size() == comps_points.size());
  ASSERT_TRUE(comps_images.size() == comps_bboxes.size());

#ifdef DISPLAY
  ppm.display(bgr, depth, comps_images);
  cv::waitKey(0);
#endif // DISPLAY

  // now test using pplp_testing
  pplp_testing::ppl_vs_user_benchmark(ppm, filename_prefix, true);
} // end direct_test();

TEST(TestSuite, direct_test_empty_lab) { direct_test(IMG_DIR "depth/empty_lab"); }
TEST(TestSuite, direct_test_all) {
  for (unsigned int i = 0; i < testvar::refset_hists_nb; ++i)
    direct_test(testvar::refset_filename_prefixes[i]);
}
TEST(TestSuite, direct_test_ainara) {
  for (unsigned int i = 0; i < testvar::ainara_hists_nb; ++i)
    direct_test(testvar::ainara_filename_prefixes[i]);
}
TEST(TestSuite, direct_test_david_arnaud1) { direct_test(IMG_DIR "depth/david_arnaud1"); }
TEST(TestSuite, direct_test_david_arnaud2) { direct_test(IMG_DIR "depth/david_arnaud2"); }
TEST(TestSuite, direct_test_david_arnaud3) { direct_test(IMG_DIR "depth/david_arnaud3"); }
#endif /////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_lab) {
  if (!rosmaster_alive()) return;
  Ppm skill;
  pplp_testing::ppl_vs_user_benchmark(skill, IMG_DIR "depth/empty_lab");
}
TEST(TestSuite, all_multi_users) {
  if (!rosmaster_alive()) return;
  Ppm skill;
  pplp_testing::ppl_vs_user_benchmark(skill, testvar::all_multi_user_filename_prefixes(),
                                     false, true, true);
}
TEST(TestSuite, all_single_users) {
  if (!rosmaster_alive()) return;
  Ppm skill;
  pplp_testing::ppl_vs_user_benchmark(skill, testvar::all_single_user_filename_prefixes(),
                                     false, true, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, start_stop) {
  if (!rosmaster_alive()) return;
  Ppm skill;
  pplp_testing::start_stop(skill);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, speed_test) {
  if (!rosmaster_alive()) return;
  Ppm skill;
  pplp_testing::speed_test(skill, false, 10, .8);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest_ppm");
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
