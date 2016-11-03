/*!
  \file        gtest_artoolkit_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/10

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

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */
// Bring in gtest
#include <gtest/gtest.h>
#include "people_detection_vision/artoolkit_utils.h"
#include <vision_utils/img_path.h>
#define ART_TEST_DIR  LONG_TERM_MEMORY_DIR "artoolkit/tests/"

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_ctor) {
  vision_utils::Id2PatternName map;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_empty) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(".dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_non_existing_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "non_existing.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_empty_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "empty.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_comment_only_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "comment_only.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_0_blocks_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "0_blocks.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_1_block_empty_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "1_block_empty.dat");
  ASSERT_TRUE(ok == false);
  ASSERT_TRUE(map.nkeys() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_1_block_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "1_block.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 1);
  // direct_lookup
  std::string pattern_name = "";
  ASSERT_TRUE(map.direct_lookup(3, pattern_name) == false);
  ASSERT_TRUE(map.direct_lookup(2, pattern_name) == false);
  ASSERT_TRUE(map.direct_lookup(1, pattern_name) == false);
  ASSERT_TRUE(map.direct_lookup(0, pattern_name) == true);
  ASSERT_TRUE(pattern_name == "4x4_2");
  // reverse_lookup
  int pattern_id = -1;
  ASSERT_TRUE(map.reverse_lookup("", pattern_id) == false);
  ASSERT_TRUE(map.reverse_lookup("foo", pattern_id) == false);
  ASSERT_TRUE(map.reverse_lookup("4x4_2_", pattern_id) == false);
  ASSERT_TRUE(map.reverse_lookup("_4x4_2", pattern_id) == false);
  ASSERT_TRUE(map.reverse_lookup("4x4_2", pattern_id) == true);
  ASSERT_TRUE(pattern_id == 0);
}


////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, parse_2_blocks_file) {
  vision_utils::Id2PatternName map;
  bool ok = map.parse(ART_TEST_DIR "2_blocks.dat");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(map.nkeys() == 2);
  // direct_lookup
  std::string pattern_name = "";
  ASSERT_TRUE(map.direct_lookup(2, pattern_name) == false);
  ASSERT_TRUE(map.direct_lookup(0, pattern_name) == true);
  ASSERT_TRUE(pattern_name == "4x4_2");
  ASSERT_TRUE(map.direct_lookup(1, pattern_name) == true);
  ASSERT_TRUE(pattern_name == "4x4_1");
  // reverse_lookup
  int pattern_id = -1;
  ASSERT_TRUE(map.reverse_lookup("foo", pattern_id) == false);
  ASSERT_TRUE(map.reverse_lookup("4x4_2", pattern_id) == true);
  ASSERT_TRUE(pattern_id == 0);
  ASSERT_TRUE(map.reverse_lookup("4x4_1", pattern_id) == true);
  ASSERT_TRUE(pattern_id == 1);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


