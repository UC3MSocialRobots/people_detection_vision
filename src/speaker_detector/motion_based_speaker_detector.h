/*!
  \file        motion_based_speaker_detector.h
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

\todo Description of the file

 */

#ifndef MOTION_BASED_SPEAKER_DETECTOR_H
#define MOTION_BASED_SPEAKER_DETECTOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "combinatorics/assignment_utils.h"

typedef int NonVerbalActivity;
static const NonVerbalActivity UNKNOWN_ACTIVITY = -1;

namespace GU = geometry_utils;

//! shorthand for a rectangle center
inline cv::Point rect_center(const cv::Rect & r) {
  return geometry_utils::rect_center<cv::Rect, cv::Point>(r);
}

////////////////////////////////////////////////////////////////////////////////

inline void draw_activity(cv::Mat & img,
                          const NonVerbalActivity activity) {
  cv::putText(img, StringUtils::cast_to_string(activity),
              cv::Point(img.cols / 2, 20), CV_FONT_HERSHEY_PLAIN,
              1, CV_RGB(255, 255, 255));
}

////////////////////////////////////////////////////////////////////////////////

//! minimal versopm for manually aligned faces
NonVerbalActivity non_verbal_activity(const cv::Mat & prev_subface,
                                      const cv::Mat & curr_subface,
                                      cv::Mat & buffer) {
  if (prev_subface.size() != curr_subface.size()) {
    printf("non_verbal_activity(): "
           "prev_subface of size %ix%i != curr_subface %ix%i\n",
           prev_subface.cols, prev_subface.rows,
           curr_subface.cols, curr_subface.rows);
    return UNKNOWN_ACTIVITY;
  }
  // threshold
  cv::absdiff(curr_subface, prev_subface, buffer);
  cv::cvtColor(buffer, buffer, CV_BGR2GRAY);
  cv::threshold(buffer, buffer, 10, 255, CV_THRESH_BINARY);

  // estimate number of different pixels
  int diff_pixels = cv::countNonZero(buffer);
  double ratio_diff = 1.f * diff_pixels / (buffer.cols * buffer.rows);
  int ratio_diff_int = 100 * ratio_diff;
  return ratio_diff_int;
} // end non_verbal_activity()

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief non_verbal_activity
 * \param curr_face
 * \param curr_roi
 * \param prev_face
 * \param prev_roi
 * \param buffer
 * \return
 *    UNKNOWN_ACTIVITY: fail in computing
 *    0: face completely static
 *    100: face in full motion
 */
NonVerbalActivity non_verbal_activity(const cv::Mat & prev_face,
                                      const cv::Rect & prev_roi,
                                      const cv::Mat & curr_face,
                                      const cv::Rect & curr_roi,
                                      cv::Mat & buffer) {
  // compute intersection of ROI
  cv::Rect roi_inter = geometry_utils::rectangle_intersection(curr_roi, prev_roi);
  if (roi_inter.width == 0 || roi_inter.height == 0) {
    printf("is_speaking(): ROIs have an empty intersection\n");
    return UNKNOWN_ACTIVITY;
  }

  // convert into coordinates inside curr_face and prev_face
  cv::Rect curr_roi_inter(roi_inter.x - curr_roi.x,
                          roi_inter.y - curr_roi.y,
                          roi_inter.width,
                          roi_inter.height);
  cv::Rect prev_roi_inter(roi_inter.x - prev_roi.x,
                          roi_inter.y - prev_roi.y,
                          roi_inter.width,
                          roi_inter.height);
  //printf("curr_roi:%s, prev_roi:%s, roi_inter:%s, "
  //       "curr_roi_inter:%s, prev_roi_inter:%s\n",
  //       GU::print_rect(curr_roi).c_str(), GU::print_rect(prev_roi).c_str(),
  //       GU::print_rect(roi_inter).c_str(),
  //       GU::print_rect(curr_roi_inter).c_str(), GU::print_rect(prev_roi_inter).c_str());
  // get subimages
  try {
    const cv::Mat curr_subface = curr_face(curr_roi_inter),
        prev_subface = prev_face(prev_roi_inter);
    // cv::imshow("curr_subface", curr_subface);
    // cv::imshow("prev_subface", prev_subface);
    //cv::imshow("buffer", buffer);
    return non_verbal_activity(prev_subface, curr_subface, buffer);
  }
  catch (cv::Exception e) {
    printf("is_speaking: exception '%s'\n", e.what());
    return UNKNOWN_ACTIVITY;
  }
} // end non_verbal_activity()

////////////////////////////////////////////////////////////////////////////////

void non_verbal_activity(const std::vector<cv::Mat> & prev_faces,
                         const std::vector<cv::Rect> & prev_faces_rois,
                         const std::vector<cv::Mat> & curr_faces,
                         const std::vector<cv::Rect> & curr_faces_rois,
                         std::vector<NonVerbalActivity> & ans,
                         cv::Mat & buffer) {

  unsigned int curr_nfaces = curr_faces_rois.size(),
      prev_nfaces = prev_faces_rois.size();

  // init answer
  ans.resize(curr_nfaces);
  for (unsigned int face_idx = 0; face_idx < curr_nfaces; ++face_idx)
    ans[face_idx] = UNKNOWN_ACTIVITY;

  // compute distance between each pair of (prev, curr) ROI centers
  CMatrix<double> costs(prev_nfaces, curr_nfaces);
  for (unsigned int prev_face_idx = 0; prev_face_idx < prev_nfaces; ++prev_face_idx) {
    for (unsigned int curr_face_idx = 0; curr_face_idx < curr_nfaces; ++curr_face_idx) {
      double dist = geometry_utils::distance_points_squared
                    (rect_center(prev_faces_rois[prev_face_idx]),
                     rect_center(curr_faces_rois[curr_face_idx]));
      costs[prev_face_idx][curr_face_idx] = dist;
    } // end loop curr_face_idx
  } // end loop prev_face_idx

  // make assignment
  assignment_utils::MatchList best_assign;
  assignment_utils::Cost best_cost;
  assignment_utils::linear_assign(costs, best_assign, best_cost);

  // compute activity for each assignment
  for (unsigned int assign_idx = 0; assign_idx < best_assign.size(); ++assign_idx) {
    assignment_utils::Match curr_assign = best_assign[assign_idx];
    if (curr_assign.first == assignment_utils::UNASSIGNED ||
        curr_assign.second == assignment_utils::UNASSIGNED)
      continue;
    ans[curr_assign.second] = non_verbal_activity(prev_faces[curr_assign.first],
        prev_faces_rois[curr_assign.first],
        curr_faces[curr_assign.second],
        curr_faces_rois[curr_assign.second],
        buffer);
  } // end loop assign_idx
} // end non_verbal_activity()

#endif // MOTION_BASED_SPEAKER_DETECTOR_H
