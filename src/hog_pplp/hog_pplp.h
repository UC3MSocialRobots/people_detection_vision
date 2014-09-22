/*!
  \file        hog_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/23

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

\class HogPPLP
\brief A \a RgbDepthPPLPublisherTemplate finding people in a color image.

cf <a href="http://experienceopencv.blogspot.com/2011/02/hog-descriptor.html">
http://experienceopencv.blogspot.com/2011/02/hog-descriptor.html
</a>

\section Parameters
  - \b hog_resize_scale
    [double] (default: 0.4)
    The scaling to make on the frame, to make HOG computation faster.

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

#ifndef HOG_PPLP_H
#define HOG_PPLP_H

// OpenCV
#include "opencv2/objdetect/objdetect.hpp"
// AD
#include <geom/Rect3.h>
#include <time/timer.h>
#include "image_utils/io.h"
#include "point_clouds/image_clusterer.h"
// people_msgs
#include "templates/rgb_depth_pplp_template.h"
#include "ppl_utils/images2pp.h"

class HogPPLP : public RgbDepthPPLPublisherTemplate  {
public:

  //////////////////////////////////////////////////////////////////////////////

  HogPPLP() : RgbDepthPPLPublisherTemplate("HOG_PPLP_START",
                                           "HOG_PPLP_STOP")
  {
    _hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    //_hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

    // parameters
    _nh_private.param("hog_resize_scale", _hog_resize_scale, 0.4);
    _nh_private.param("hog_scale_step", _hog_scale_step, 1.05);
    _nh_private.param("hog_hitThreshold", _hog_hitThreshold, 2.0);
    _nh_private.param("hog_finalThreshold", _hog_finalThreshold, 2.0);
    _nh_private.param("clusterer_data_skip", _clusterer_data_skip, 10);

    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    kinect_openni_utils::read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, rgb_camera_model);

    DEBUG_PRINT("HogPPLP: started with '%s' and stopped with '%s', "
                "subscribing to '%s', '%s', "
                "publish PeoplePoseList results on '%s'\n",
                get_start_stopic().c_str(), get_stop_stopic().c_str(),
                get_rgb_topic().c_str(), get_depth_topic().c_str(),
                get_ppl_topic().c_str());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  //! this function is called each time an image is received
  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {
    DEBUG_PRINT("process_rgb_depth()\n");

    Timer timer;
    // clear previous data
    _cuts_offsets.clear();
    _rgb_cuts.clear();
    _depth_cuts.clear();
    _user_cuts.clear();
    _faces_centers3d.clear();

    // prepare the small image
    cv::resize(rgb, _small_img_color, cv::Size(0, 0),
               _hog_resize_scale, _hog_resize_scale, cv::INTER_NEAREST);
    cv::cvtColor(_small_img_color, _small_img_bw, CV_BGR2GRAY);

    // run the detector with default parameters.
    // To get a higher hit-rate (and more false alarms, respectively),
    // decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    std::vector<cv::Rect> found_rectangles;
    _hog.detectMultiScale(_small_img_bw, // const Mat& img, - Matrix of the type CV_8U containing an image where objects are detected.
                          found_rectangles,  // CV_OUT vector<Rect>& foundLocations,
                          _hog_hitThreshold, // double hitThreshold=0
                          cv::Size(8,8), // Size winStride=Size()
                          cv::Size(32,32), // Size padding=Size()
                          _hog_scale_step, // double scale=1.05
                          _hog_finalThreshold // double finalThreshold=2.0
                          );
    DEBUG_PRINT("Time for _hog.detectMultiScale():%g ms\n", timer.time());

    // filter the rectangles
    geometry_utils::remove_including_rectangles
        (found_rectangles, _found_rectangles_filtered);
    unsigned int nusers = _found_rectangles_filtered.size();

    // rescale found faces
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      cv::Rect* r = &(_found_rectangles_filtered[user_idx]);
      r->x /= _hog_resize_scale;
      r->y /= _hog_resize_scale;
      r->width /= _hog_resize_scale;
      r->height /= _hog_resize_scale;
      // the HOG detector returns slightly larger rectangles than the real objects.
      // so we slightly shrink the rectangles to get a nicer output.
      //*r = geometry_utils::shrink_rec(*r, 0.8);
      // only shrink horizontally
      double ratio = .5;
      r->x = r->x + r->width * (1. - ratio) / 2;
      r->width = ratio * r->width;
      // ensure the new rectangle is in the image
      *r = geometry_utils::rectangle_intersection_img(rgb, *r);
    } // end loop user_idx
    // DEBUG_PRINT("Time for rectangles rescaling:%g ms\n", timer.time());

    DEBUG_PRINT("Time for rectangles filtering and rescaling:%g ms\n", timer.time());

    _cuts_offsets.reserve(nusers);
    _rgb_cuts.reserve(nusers);
    _depth_cuts.reserve(nusers);
    _user_cuts.reserve(nusers);
    _faces_centers3d.reserve(nusers);
    bool can_pixe2world = true;
    if (!can_pixe2world) {
      DEBUG_PRINT("Cannot convert the found rectangles to 3D, "
                  "can_pixe2world:%i, depth.empty():%i",
                  can_pixe2world, depth.empty());
    } // end if (!can_pixe2world)
    else { // can_pixe2world
      for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
        cv::Mat3b rgb_cut;
        cv::Mat1f depth_cut;
        cv::Mat1b user_cut;
        cv::Point3d face_center3d;
        if (!compute_cluster_roi(rgb, depth, _found_rectangles_filtered[user_idx],
                                 rgb_cut, depth_cut, user_cut, face_center3d))
          continue;
        _cuts_offsets.push_back(_found_rectangles_filtered[user_idx].tl());
        _rgb_cuts.push_back(rgb_cut);
        _depth_cuts.push_back(depth_cut);
        _user_cuts.push_back(user_cut);
        _faces_centers3d.push_back(face_center3d);
      }
      DEBUG_PRINT("Time for clusterer.cluster():%g ms\n", timer.time());

      build_ppl_message();
    } // end if (can_pixe2world)

    DEBUG_PRINT("Time for process_rgb_depth():%g m, %i users\n", timer.time(), nusers);
  } // end process_rgb_depth();

  //////////////////////////////////////////////////////////////////////////////

  bool compute_cluster_roi(const cv::Mat3b & rgb,
                           const cv::Mat1f & depth,
                           const cv::Rect & roi,
                           cv::Mat3b & rgb_cut,
                           cv::Mat1f & depth_cut,
                           cv::Mat1b & user_cut,
                           cv::Point3d & face_center3d) {
    // DEBUG_PRINT("compute_cluster_roi(%s)\n", geometry_utils::print_rect(roi).c_str());
    if (!_clusterer.cluster(rgb(roi), depth(roi), _default_depth_camera_model, _clusterer_data_skip)) {
      printf("Fail in clusterer.cluster()!\n");
      return false;
    }
    // get the bounding box -> face_center3d
    geometry_utils::Rect3d bbox;
    if (!_clusterer.get_biggest_cluster_bbox(bbox)) {
      printf("Fail in clusterer.get_biggest_cluster_bbox()!\n");
      return false;
    }
    face_center3d.x = bbox.x + bbox.width/2;
    // head.y = 10 cm under top (y axis is inverted)
    face_center3d.y = bbox.y + 0.10;
    face_center3d.z = bbox.z + bbox.depth/2;
    // printf("face_center3d:'%s'\n", geometry_utils::printP(face_center3d).c_str());

    // generate mask
    if (!_clusterer.get_biggest_cluster_pixels
        (_default_depth_camera_model, _cluster_pixels, _cluster_colors)) {
      printf("Fail in clusterer.get_biggest_cluster_pixels()!\n");
      return false;
    }
    user_cut.create(cv::Size(roi.width, roi.height));
    user_cut.setTo(0);
    unsigned int npixels = _cluster_pixels.size();
    cv::Point offset = roi.tl();
    for (unsigned int pt_idx = 0; pt_idx < npixels; ++pt_idx)
      cv::circle(user_cut, _cluster_pixels[pt_idx] - offset, _clusterer_data_skip, CV_RGB(255, 255, 255), -1);
    rgb(roi).copyTo(rgb_cut, user_cut);
    depth(roi).copyTo(depth_cut, user_cut);

    return true;
  } // end compute_cluster_roi();

  //////////////////////////////////////////////////////////////////////////////

  /*! share the poses of the detected users in a
   *  people_msgs::PeoplePoseList msg */
  void build_ppl_message() {
    // share the poses
    unsigned int n_faces = _faces_centers3d.size();
    assert(_rgb_cuts.size() == n_faces);
    assert(_depth_cuts.size() == n_faces);
    assert(_user_cuts.size() == n_faces);
    assert(_cuts_offsets.size() == n_faces);
    _ppl.header = _images_header; // reuse the header of the last frame
    _ppl.method = "hog_pplp";
    _ppl.poses.resize(n_faces);

    // build the PeoplePose
    for (unsigned int user_idx = 0; user_idx < n_faces; ++user_idx) {
      people_msgs::PeoplePose* pp = &(_ppl.poses[user_idx]);
      pp->header = _ppl.header; // copy header
      // pp->person_name = StringUtils::cast_to_string(user_idx);
      pp->person_name = people_msgs::PeoplePose::NO_RECOGNITION_MADE;
      pp->confidence = 1;

      // pose
      pt_utils::copy3(_faces_centers3d[user_idx], pp->head_pose.position);
      pp->head_pose.orientation = tf::createQuaternionMsgFromYaw(0);
      pp->std_dev = .1;

      // image
      _images2pp.convert(*pp, &(_rgb_cuts[user_idx]), &(_depth_cuts[user_idx]), &(_user_cuts[user_idx]), false);
      // restore proper offset (images were crops)
      pp->images_offsetx = _cuts_offsets[user_idx].x;
      pp->images_offsety = _cuts_offsets[user_idx].y;
      //  printf("PP #%i:(%i, %i)+(%i, %i)\n", user_idx,
      //         pp->images_offsetx, pp->images_offsety, pp->user.width, pp->user.height);
    } // end loop user_idx
    publish_PPL(_ppl);
  } // end build_ppl_message()

  //////////////////////////////////////////////////////////////////////////////

  virtual void display(const cv::Mat3b & rgb,
                       const cv::Mat1f & depth) {
    cv::imshow("rgb", rgb);
    cv::imshow("depth", image_utils::depth2viz(depth, image_utils::FULL_RGB_STRETCHED));
    rgb.copyTo(img_out);

    for(unsigned int user_idx = 0; user_idx < _found_rectangles_filtered.size(); user_idx++) {
      cv::Rect roi = _found_rectangles_filtered[user_idx];
      // paint the masks
      img_out(roi).setTo(cv::Vec3b(255, 0, 0), _user_cuts[user_idx]);
      // paint the found rectangles
      cv::rectangle(img_out, roi, CV_RGB(255,0,0), 3);
      // paint the faces centers
      //  cv::circle(img_out, world2pixel_depth(_faces_centers3d[user_idx]),
      //             8, CV_RGB(255, 0, 0), -1);
    }

    cv::imshow("HogPPLP", img_out);
  } // end display();

  //////////////////////////////////////////////////////////////////////////////

private:
  //! the redim image
  cv::Mat3b _small_img_color;
  cv::Mat1b _small_img_bw;
  //! an image for drawing stuff
  cv::Mat3b img_out;

  //! the hog finder
  cv::HOGDescriptor _hog;
  //! the scale reduction factor of the RGB image for face detection
  double _hog_resize_scale, _hog_scale_step, _hog_hitThreshold, _hog_finalThreshold;
  std::vector<cv::Rect> _found_rectangles_filtered;

  //! the list of faces, with ROS orientation, in RGB frame
  std::vector< cv::Point > _cuts_offsets;
  std::vector< cv::Point3d > _faces_centers3d;
  std::vector<cv::Mat3b> _rgb_cuts;
  std::vector<cv::Mat1f> _depth_cuts;
  std::vector<cv::Mat1b> _user_cuts;

  //! clustering
  int _clusterer_data_skip;
  std::vector<cv::Point2i> _cluster_pixels;
  std::vector<cv::Scalar> _cluster_colors;
  ImageClusterer _clusterer;
  image_geometry::PinholeCameraModel _default_depth_camera_model;

  //! information sharing
  people_msgs::PeoplePoseList _ppl;
  ppl_utils::Images2PP _images2pp;
}; // end class HogPPLP

#endif // HOG_PPLP_H
