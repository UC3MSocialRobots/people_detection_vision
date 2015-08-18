/*!
  \file        tabletop_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/2

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

A user detector working with the depth image.
It detect blobs in the depth image using a Canny filter.
Optionnaly, the ground plane can be detecting using a RANSAC estimator.
Other parameters can fine-tune the detector,
such as a minimum size for the blobs.

\section Parameters
  - \b "~rgb_topic", "~depth_topic"
        [std::string, default "rgb", "depth"]
        \see RgbDepthPPLPublisherTemplate

  - \b "~min_dist_m", "~max_dist_m"
        [double, default -1, -1]
        the range of depth values where we will look for users, in meters.
        -1 for not using this filter.

  - \b "~max_blobs_nb"
        [int, default 30]
        the maximum allowed number of users.
        the real number of found users is in [0, max_blobs_nb].
        -1 for not using this filter.

  - \b "~min_blob_size_pix"
        [int, default 3000]
        the minimum size in pixels for the found blobs.
        The smaller blobs will be dismissed.
        -1 for not using this filter.

\section Subscriptions
  - \b {start_topic}, {stop_topic}
        [std_msgs::Int16]
        \see PPLPublisherTemplate.

  - \b {rgb_topic}, {depth_topic}
        [sensor_msgs::Image]
        \see RgbDepthPPLPublisherTemplate

\section Publications
  - \b "~ppl"
        [people_msgs::PeoplePoseList]
        The detected users, \see PPLPublisherTemplate.
 */

#ifndef TABLETOP_PPLP_H
#define TABLETOP_PPLP_H

// people_msgs
#include <templates/rgb_depth_pplp_template.h>
#include "ppl_utils/images2ppl.h"
#include "vision_utils/color_utils.h"
#include "vision_utils/blob_segmenter.h"
#include "vision_utils/content_processing.h"
#include "vision_utils/drawing_utils.h"
#include <ros_utils/pt_utils.h>

class TabletopPPLP : public RgbDepthPPLPublisherTemplate {
public:
  TabletopPPLP() : RgbDepthPPLPublisherTemplate("TABLETOP_PPLP_START",
                                                "TABLETOP_PPLP_STOP") {

    // get params
    _nh_private.param("min_dist_m", min_dist_m, (double) -1);
    _nh_private.param("max_dist_m", max_dist_m, (double) -1);
    _nh_private.param("max_blobs_nb", max_blobs_nb, 30);
    _nh_private.param("min_blob_size_pix", min_blob_size_pix, 3000);

    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    kinect_openni_utils::read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, rgb_camera_model);

    DEBUG_PRINT("TabletopPPLP: started with '%s' and stopped with '%s', "
                "subscribing to '%s', '%s', "
                "publish PeoplePoseList results on '%s'\n",
                get_start_stopic().c_str(), get_stop_stopic().c_str(),
                get_rgb_topic().c_str(), get_depth_topic().c_str(),
                get_ppl_topic().c_str());
  } //end ctor

  //////////////////////////////////////////////////////////////////////////////

  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {
    // DEBUG_PRINT("process_rgb_depth()\n");
    bool ground_recompute_plane = true;
    _segmenter.find_all_blobs(depth, _components_pts, boundingBoxes,
                              BlobSegmenter::GROUND_PLANE_FINDER,
                              &_default_depth_camera_model,
                              min_dist_m, max_dist_m,
                              max_blobs_nb, min_blob_size_pix,
                              ground_recompute_plane);
    // build PPL message
    int nusers = _components_pts.size();
    curr_user_mask.create(depth.size());
    curr_user_mask.setTo(0);
    _ppl.header = _images_header; // reuse the header of the last frame
    _ppl.method = "tabletop_pplp";
    _ppl.poses.resize(nusers);
    for (int user_idx = 0; user_idx < nusers; ++user_idx) {
      people_msgs::PeoplePose* pp = &(_ppl.poses[user_idx]);
      cv::Rect curr_roi = boundingBoxes[user_idx];
      pp->header = _ppl.header; // copy header
      pp->person_name = people_msgs::PeoplePose::NO_RECOGNITION_MADE;
      pp->confidence = 1;

      // image
      image_utils::drawListOfPoints(curr_user_mask, _components_pts[user_idx], (uchar) 255);
      cv::Mat3b curr_rgb_roi = rgb(curr_roi);
      cv::Mat1f curr_depth_roi = depth(curr_roi);
      cv::Mat1b curr_user_roi = curr_user_mask(curr_roi);
      _images2pp.convert(*pp, &curr_rgb_roi, &curr_depth_roi, &curr_user_roi, false);

      // pose - need to be done after image for the use of curr_user_mask
      cv::Point2d user_center2d = geometry_utils::rect_center<cv::Rect, cv::Point>(curr_roi);
      user_center2d = _roi_center2user_mask.find(curr_user_mask, user_center2d);
      if (user_center2d.x < 0) {
        printf("TabletopPPLP:user %i: cant find a user point in the mask!\n",
               user_idx);
        //continue;
      }
      cv::Point3d user_center3d = kinect_openni_utils::pixel2world_depth<cv::Point3d>
                                  (user_center2d, _default_depth_camera_model, depth);
      pt_utils::copy3(user_center3d, pp->head_pose.position);
      pp->head_pose.orientation = tf::createQuaternionMsgFromYaw(0);
      pp->std_dev = .1;

      curr_user_roi.setTo(0); // clean curr_user_mask only where needed
    } // end loop user_idx
    publish_PPL(_ppl);
  } // end process_rgb_depth();

  //////////////////////////////////////////////////////////////////////////////

  void display(const cv::Mat3b & rgb,
               const cv::Mat1f & depth) {
    image_utils::depth_image_to_vizualisation_color_image(depth, _depth2viz);
    _comps_illus.create(rgb.size());
    _comps_illus.setTo(0);
    for (unsigned int comp_idx = 0; comp_idx < _components_pts.size(); ++comp_idx)
      image_utils::drawListOfPoints(_comps_illus, _components_pts[comp_idx],
                                    color_utils::color<cv::Vec3b>(comp_idx));
    // generate a mask for valid depth measures
    cv::threshold(depth, _depth_mask_buffer, min_dist_m, 1, CV_THRESH_BINARY);
    _depth_mask_illus = (_depth_mask_buffer > 0);
    cv::threshold(depth, _depth_mask_buffer, max_dist_m, 1,CV_THRESH_BINARY_INV);
    _depth_mask_illus = _depth_mask_illus & (_depth_mask_buffer > 0);

    if (_segmenter.generate_ground_mask(depth, _ground_mask_illus,
                                        &_default_depth_camera_model, min_dist_m, max_dist_m)) {
      cv::imshow("ground_mask", _ground_mask_illus);
      image_utils::imwrite_debug("ground_mask.png", _ground_mask_illus,
                                 image_utils::MONOCHROME);
    }
    cv::imshow("rgb", rgb);
    //    image_utils::imwrite_debug("rgb.png", rgb,
    //                               image_utils::COLOR_24BITS);
    cv::imshow("depth", _depth2viz);
    //    image_utils::imwrite_debug("depth.png", _depth2viz,
    //                               image_utils::COLORS256);
    cv::imshow("comps_illus", _comps_illus);
    //    image_utils::imwrite_debug("comps_illus.png", _comps_illus,
    //                               image_utils::COLORS256);
    cv::imshow("segmenter_final_mask", _segmenter.get_final_mask());
    //    image_utils::imwrite_debug("segmenter_final_mask.png", _segmenter.get_final_mask(),
    //                               image_utils::MONOCHROME);
    cv::imshow("depth_mask_illus", _depth_mask_illus);
    //    image_utils::imwrite_debug("depth_mask_illus.png", _depth_mask_illus,
    //                               image_utils::MONOCHROME);
  } // end display();

  //////////////////////////////////////////////////////////////////////////////

  //! segmentation
  BlobSegmenter _segmenter;
  std::vector< DisjointSets2::Comp > _components_pts;
  std::vector<cv::Rect> boundingBoxes;
  image_geometry::PinholeCameraModel _default_depth_camera_model;
  double min_dist_m, max_dist_m;
  int max_blobs_nb, min_blob_size_pix;

  //! information sharing
  people_msgs::PeoplePoseList _ppl;
  ppl_utils::Images2PP _images2pp;
  image_utils::ClosestPointInMask2<uchar> _roi_center2user_mask;
  cv::Mat1b curr_user_mask;

  //! illus
  cv::Mat3b _comps_illus, _depth2viz;
  cv::Mat1f _depth_mask_buffer;
  cv::Mat1b _ground_mask_illus, _depth_mask_illus;
}; // end class TabletopPPLP

#endif // TABLETOP_PPLP_H
