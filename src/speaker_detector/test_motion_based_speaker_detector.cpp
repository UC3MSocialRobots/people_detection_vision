/*!
  \file        test_motion_based_speaker_detector.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/6/28
  
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

Some tests for motion_based_speaker_detector.h

 */

#include "opencv2/highgui/highgui.hpp"
#include "motion_based_speaker_detector.h"
#include "vision_utils/image_utils/opencv_face_detector.h"
#include "src/time/timer.h"

void test_non_verbal_activity_2_images(const std::string & file1,
                                       const std::string & file2) {
  cv::Mat im1 = cv::imread(file1), im2 = cv::imread(file2);
  // find faces
  cv::Mat3b small_img;
  std::vector<cv::Rect> curr_rois, prev_rois;
  // create a classifier
  cv::CascadeClassifier classifier = image_utils::create_face_classifier();
  image_utils::detect_with_opencv(im1, classifier, small_img, curr_rois);
  image_utils::detect_with_opencv(im2, classifier, small_img, prev_rois);
  printf("%i faces in im1, %i in im2\n", curr_rois.size(), prev_rois.size());
  if (curr_rois.size() != 1 || prev_rois.size() != 1)
    return;

  // estimate if non_verbal_activity
  cv::Mat buffer;
  cv::Rect r1 = curr_rois.front(), r2 = prev_rois.front();
  NonVerbalActivity activity = non_verbal_activity(im1(r1), r1, im2(r2), r2, buffer);
  printf("activity:%i\n", activity);
  cv::imshow("buffer", buffer);
  cv::imshow("im1", im1);
  cv::imshow("im2", im2);
  cv::waitKey(0);
} // end test_non_verbal_activity_2_images();

////////////////////////////////////////////////////////////////////////////////

// #define ONE_FACE_FILTER

void test_non_verbal_activity_camera() {
  cv::VideoCapture capture(0);
  cv::Mat3b frame, prev_frame;
  cv::Mat buffer;
  // create a classifier
  cv::CascadeClassifier classifier = image_utils::create_face_classifier();
  cv::Mat3b small_img;
  std::vector<cv::Rect> curr_rois, prev_rois;
  std::vector<cv::Mat> curr_faces, prev_faces;
  while (capture.isOpened()) {
    capture >> frame;
    // make the real detection
    image_utils::detect_with_opencv(frame, classifier, small_img, curr_rois);
    // store faces
    curr_faces.clear();
    for (unsigned int idx = 0; idx < curr_rois.size(); ++idx)
      curr_faces.push_back(frame(curr_rois[idx]));

    // evaluate user speaking
    Timer timer;
    std::vector<NonVerbalActivity> activities;
#ifdef ONE_FACE_FILTER
    if (curr_rois.size() == 1 && prev_rois.size() == 1) {
      ans.push_back(non_verbal_activity(prev_faces[0], prev_rois[0],
          curr_faces[0], curr_rois[0],
          buffer));
      //ROS_INFO_THROTTLE(1, "time for non_verbal_activity():%g ms", timer.time());
    }
#else // not ONE_FACE_FILTER
    non_verbal_activity(prev_faces, prev_rois, curr_faces, curr_rois, activities, buffer);
#endif // ONE_FACE_FILTER
    timer.printTime("non_verbal_activity");

    // store data
    frame.copyTo(prev_frame);
    prev_rois = curr_rois;
    prev_faces.clear();
    for (unsigned int face_idx = 0; face_idx < curr_faces.size(); ++face_idx)
      prev_faces.push_back(curr_faces[face_idx].clone());

    // draw rectangles
    for (unsigned int face_idx = 0; face_idx < curr_rois.size(); ++face_idx)
      cv::rectangle(frame, curr_rois[face_idx], CV_RGB(0, 255, 0), 2);
    // draw activities
    for (unsigned int acti_idx = 0; acti_idx < activities.size(); ++acti_idx) {
      cv::putText(frame, StringUtils::cast_to_string(activities[acti_idx]),
                  rect_center(curr_rois[acti_idx]),
                  CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(0, 255, 0));
    } // end loop acti_idx

    // display
    cv::imshow("frame", frame);
    cv::waitKey(5);
  } // end while (capture.isOpened)
} // end test_non_verbal_activity_camera();

////////////////////////////////////////////////////////////////////////////////

int main() {
  // test_non_verbal_activity_2_images(IMG_DIR "arnaud003.jpg", IMG_DIR "arnaud004.jpg");
  test_non_verbal_activity_camera();
  return 0;
}
