/*!
  \file        gtest_nite_user_mask_pplp.cpp
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

Some tests for

\todo Description of the file
 */
//#define DISPLAY

#include <ros_utils/rosmaster_alive.h>
#include "templates/pplp_testing.h"
// people_msgs
#include "vision_utils/test_person_histogram_set_variables.h"
#include "depth_people_detector/nite_user_mask_pplp.h"
// opencv
#include <opencv2/highgui/highgui.hpp>

namespace testvar = test_person_histogram_set_variables;

////////////////////////////////////////////////////////////////////////////////

void direct_test(NiteUserMask2Ppl & skill,
          const std::string & filename_prefix) {
  // read user, depth, rgb files
  cv::Mat rgb, depth;
  cv::Mat1b user_mask;
  image_utils::read_rgb_depth_user_image_from_image_file
      (filename_prefix, &rgb, &depth, &user_mask);
  // cv::Mat3b user_illus;  user_image_to_rgb(user_mask, user_illus);  cv::imshow("NiteUserMask2Ppl", user_illus); cv::waitKey(0);
  // test the skill
  skill.process_rgb_depth_user(rgb, depth, user_mask);
#ifdef DISPLAY
  skill.display(rgb, depth, user_mask);
  cv::waitKey(0);
#endif // DISPLAY
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, direct_tests) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  skill.create_subscribers_and_publishers();
  direct_test(skill, IMG_DIR "depth/juggling1");
  direct_test(skill, IMG_DIR "depth/david_arnaud1");
  skill.shutdown_subscribers_and_publishers();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, start_stop) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  pplp_testing::start_stop(skill, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_lab) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  pplp_testing::ppl_vs_user_benchmark(skill, IMG_DIR "depth/empty_lab", true);
}
TEST(TestSuite, all_multi_users) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  pplp_testing::ppl_vs_user_benchmark(skill, testvar::all_multi_user_filename_prefixes(),
                                     true, true, true);
}
TEST(TestSuite, all_single_users) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  pplp_testing::ppl_vs_user_benchmark(skill, testvar::all_single_user_filename_prefixes(),
                                     true, true, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, speed_test) {
  if (!rosmaster_alive()) return;
  NiteUserMask2Ppl skill;
  pplp_testing::speed_test(skill, true, 30, .8);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest_skill");
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
