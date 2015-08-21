/*!
  \file        bg_substractor_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/09

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

\class BackgroundSubstractorPPLP
\brief This PPLP detects users in the depth image using a simple background
substraction.
The background model can be loaded from a static image.
It can also be updated at each frame.
The size of the blobs (in pixels) to keep can be configured,
and also the minimum and maximum position of the upper points of the blob:
this can be useful to discriminate between users and other objects for instance.

\section Parameters
  - \b "~update_background"
        [bool, default: true]
        If true, the background model will be updated upon reception of
        each depth frame.
        Points further in the current

  - \b "~min_comp_size"
        [int (pixels), default: 200 pixels]
        The minimal size for a connected component to be considered to be an object.

  - \b "~min_comp_z, max_comp_z"
        [double (meters), default: -10 and 10]
        The minimal and maximal values of the z component
        for a connected component to be considered to be an object.

  - \b "~static_frame"
        [string, default:"/floor"]
        The static frame in which min_comp_z and max_comp_z are defined.

  - \b "~canny_thres1, canny_thres2"
        [double (meters), default: DepthCanny::DEFAULT_CANNY_THRES1/2]
        \see DepthCanny::set_canny_thresholds()
        Decrease to make more edges appear.

  - \b "~depth_bg_model_filename_prefix"
        [std::string, default:""]
        Pre-load a background model from an image file on the disk.
        If empty, do not pre-load anything, use the first acquired depth image as model.

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
#ifndef BG_SUBSTRACTOR_PPLP_H
#define BG_SUBSTRACTOR_PPLP_H

// AD
#include "vision_utils/cv_conversion_float_uchar.h"

#include "vision_utils/kinect_openni_utils.h"
#include "vision_utils/utils/pt_utils.h"
#include "vision_utils/utils/timestamp.h"
#include "vision_utils/color_utils.h"
#include "vision_utils/disjoint_sets2.h"
#include "vision_utils/depth_canny.h"
#include "vision_utils/drawing_utils.h"
#include "vision_utils/head_finder.h"
// people_msgs
#include "vision_utils/rgb_depth_pplp_template.h"
#include "vision_utils/images2ppl.h"
// ROS
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#undef DEBUG_PRINT
#define DEBUG_PRINT(...)   printf(__VA_ARGS__)


class BackgroundSubstractorPPLP : public RgbDepthPPLPublisherTemplate  {
public:
  /*! the difference between a pixel depth and the background depth
  in meters to be considered as foreground */
  static const double FOREGROUND_MIN_DEPTH_RATIO = .9;

  //////////////////////////////////////////////////////////////////////////////

  BackgroundSubstractorPPLP() : RgbDepthPPLPublisherTemplate("BG_SUBSTRACTOR_PPLP_START",
                                                             "BG_SUBSTRACTOR_PPLP_STOP")
  {
    _tf_listener = new tf::TransformListener();

    // parameters
    std::string bg_prefix = "";
    _nh_private.param("depth_bg_model_filename_prefix", bg_prefix, bg_prefix);
    _update_background = true;
    _nh_private.param("update_background", _update_background, _update_background);
    _min_comp_size = 200;
    _nh_private.param("min_comp_size", _min_comp_size, _min_comp_size);
    _static_frame = "/floor";
    _nh_private.param("static_frame", _static_frame, _static_frame);
    _nh_private.param("min_comp_z", _min_comp_z, -10.);
    _nh_private.param("max_comp_z", _max_comp_z, 10.);
    double canny_thres1, canny_thres2;
    const double DEFAULT_CANNY_THRES1_auxConst = DepthCanny::DEFAULT_CANNY_THRES1;
    const double DEFAULT_CANNY_THRES2_auxConst = DepthCanny::DEFAULT_CANNY_THRES2;
    _nh_private.param("canny_thres1", canny_thres1, DEFAULT_CANNY_THRES1_auxConst);
    _nh_private.param("canny_thres1", canny_thres2, DEFAULT_CANNY_THRES2_auxConst);
    _canny.set_canny_thresholds(canny_thres1, canny_thres2);

    bool has_depth_bg_model = false;
    if (!bg_prefix.empty()) {
      has_depth_bg_model = image_utils::read_rgb_and_depth_image_from_image_file
          (bg_prefix, NULL, &_background);
    }

    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    kinect_openni_utils::read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, rgb_camera_model);

    std::ostringstream info;
    info << "BackgroundSubstractorPPLP: started with '" << get_start_stopic()
         << "' and stopped with '" << get_stop_stopic()
         << "', subscribing to '"
         << get_rgb_topic() << "', '" << get_depth_topic()
         << "', publish PeoplePoseList results on '" << get_ppl_topic()
         << "', update_background:" << _update_background;
    if (has_depth_bg_model)
      info << ", using depth bg model '" << bg_prefix << "'";
    else
      info << ", without depth bg model";
    if (_display)
      info << ". Press 's' in the windows to save background models to disk.";
    DEBUG_PRINT("%s\n", info.str().c_str());
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
    _faces_centers2d3d.clear();

    // update background
    // create a background from depth the first time
    if (_background.empty())
      depth.copyTo(_background);
    // foreground objects are such as background - depth > thresh
    _foreground.create(depth.size());
    _foreground.setTo(0);
    unsigned int n_pixels = depth.cols * depth.rows;
    const float* depth_ptr = depth.ptr<float>();
    float* background_ptr = _background.ptr<float>();
    uchar* foreground_ptr = _foreground.ptr();
    for (unsigned int pixel_idx = 0; pixel_idx < n_pixels; ++pixel_idx) {
      // remove from foreground the points were depth is not defined (0)
      if(!std_utils::is_nan_depth(*depth_ptr)) {
        bool background_defined = !std_utils::is_nan_depth(*background_ptr);
        if (!background_defined
            || *depth_ptr < *background_ptr * FOREGROUND_MIN_DEPTH_RATIO) { // this pixel is foreground
          *foreground_ptr = 255;
        }
        if (_update_background && // set new background
            (!background_defined || *depth_ptr > *background_ptr))
          *background_ptr = *depth_ptr;
      }
      ++depth_ptr;
      ++background_ptr;
      ++foreground_ptr;
    } // end loop pixel_idx
    //cv::morphologyEx(_foreground, _foreground, cv::MORPH_OPEN, cv::Mat(10, 10, CV_8U, 255));
    //cv::morphologyEx(_foreground, _foreground, cv::MORPH_ERODE, cv::Mat(15, 15, CV_8U, 255));
    DEBUG_PRINT("Time for foreground computation:%g ms\n", timer.time());

    // compute a depth Canny
    _canny.thresh(depth);
    // combine depth Canny and background
    // all points equal to 0 in Canny must be set to 0 in foreground
    _foreground.setTo(0, _canny.get_thresholded_image() == 0);
    DEBUG_PRINT("Time for Canny:%g ms\n", timer.time());

    // use disjoint sets to get components
    // get foreground points ROIS with disjoint sets
    _set.process_image(_foreground);
    _set.get_connected_components(_foreground.cols, _components_pts, _boundingBoxes);
    _set.sort_comps_by_decreasing_size(_components_pts, _boundingBoxes);
    DEBUG_PRINT("Time for Disjoint sets:%g ms\n", timer.time());
    unsigned int nbboxes = _boundingBoxes.size();
    _comp_was_kept.resize(nbboxes, false);

    // for each found component
    _cuts_offsets.reserve(nbboxes);
    _rgb_cuts.reserve(nbboxes);
    _depth_cuts.reserve(nbboxes);
    _user_cuts.reserve(nbboxes);
    _faces_centers2d3d.reserve(nbboxes);

    for (unsigned int bbox_idx = 0; bbox_idx < nbboxes; ++bbox_idx) {
      if ((int) _components_pts[bbox_idx].size() < _min_comp_size)
        break;
      std::vector<cv::Point>* comp = &(_components_pts[bbox_idx]);
      cv::Rect roi = _boundingBoxes[bbox_idx];
      cv::Point tl = roi.tl();
      // reproject upper point 3D
      //cv::Point2d head2D (tl.x + roi.width / 2, tl.y + roi.height * .1);
      //cv::Point2d head2D = comp->front();
      cv::Point head2i;
      cv::Point2d head2D;
      _user_mask.create(_foreground.size());
      _user_mask.setTo(0);
      image_utils::drawListOfPoints(_user_mask, *comp, (uchar) 255);
      if (!_head_finder.find(_user_mask, head2i)) {
        ROS_WARN("Could not find head for blob #%i!", bbox_idx);
        continue;
      }
      pt_utils::copy2(head2i, head2D);
      // convert to 3D in the good frame
      cv::Point3d head3D_world, head3D_cam =
          kinect_openni_utils::pixel2world_depth<cv::Point3d>
          (head2D, _default_depth_camera_model, depth);
      try {
        geometry_msgs::PoseStamped pose_cam, pose_world;
        pt_utils::copy3(head3D_cam, pose_cam.pose.position);
        pose_cam.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        pose_cam.header = _images_header;
        pose_world.header.stamp = _images_header.stamp;
        pose_world.header.frame_id = _static_frame;
        _tf_listener->transformPose(_static_frame, ros::Time(0),
                                    pose_cam, _static_frame, pose_world);
        pt_utils::copy3(pose_world.pose.position, head3D_world);
      } catch (tf::ExtrapolationException e) {
        ROS_WARN("transform error:'%s'", e.what());
        continue;
      }
      if (head3D_world.z < _min_comp_z || head3D_world.z > _max_comp_z)
        continue;
      DEBUG_PRINT("Head %i:%s\n", bbox_idx, geometry_utils::printP(head3D_world).c_str());

      // keep it in _faces_centers2d3d
      std::pair<cv::Point, cv::Point3d> new_pair(head2D, head3D_world);
      _faces_centers2d3d.push_back(new_pair);

      // if size satisfying, keep it in _rgb_cuts, _depth_cuts, _user_cuts, etc.
      _comp_was_kept[bbox_idx] = true;
      _cuts_offsets.push_back(tl);
      _rgb_cuts.push_back(rgb(roi));
      _depth_cuts.push_back(depth(roi));
      _user_cuts.push_back(_foreground(roi));
    } // end for bbox_idx

    unsigned int nusers = _faces_centers2d3d.size();
    build_ppl_message();
    DEBUG_PRINT("Time for process_rgb_depth():%g ms, %i users\n", timer.time(), nusers);
    if (_display) display(rgb, depth);
  } // end process_rgb_depth();

  //////////////////////////////////////////////////////////////////////////////

  /*! share the poses of the detected users in a
   *  people_msgs::PeoplePoseList msg */
  void build_ppl_message() {
    // share the poses
    unsigned int nfaces = _faces_centers2d3d.size();
    assert(_rgb_cuts.size() == nfaces);
    assert(_depth_cuts.size() == nfaces);
    assert(_user_cuts.size() == nfaces);
    assert(_cuts_offsets.size() == nfaces);
    _ppl.header = _images_header; // reuse the header of the last frame
    _ppl.method = "BG_SUBSTRACTOR_PPLP";
    _ppl.poses.resize(nfaces);

    // build the PeoplePose
    for (unsigned int user_idx = 0; user_idx < nfaces; ++user_idx) {
      people_msgs::PeoplePose* pp = &(_ppl.poses[user_idx]);
      pp->header = _ppl.header; // copy header
      // pp->person_name = StringUtils::cast_to_string(user_idx);
      pp->person_name = people_msgs::PeoplePose::NO_RECOGNITION_MADE;
      pp->confidence = 1;

      // pose
      pt_utils::copy3(_faces_centers2d3d[user_idx].second, pp->head_pose.position);
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
    //    cv::imshow("rgb", rgb);
    //    cv::imshow("depth", image_utils::depth2viz(depth, image_utils::FULL_RGB_STRETCHED));
    //cv::imshow("BackgroundSubstractorPPLP_foreground", _foreground);
    cv::imshow("BackgroundSubstractorPPLP_background", image_utils::depth2viz(_background, image_utils::FULL_RGB_STRETCHED));
    cv::cvtColor(_foreground, img_out, CV_GRAY2BGR);
    // paint components
    for (unsigned int comp_idx = 0; comp_idx < _components_pts.size(); ++comp_idx) {
      if (_comp_was_kept[comp_idx])
        image_utils::drawListOfPoints
            (img_out, _components_pts[comp_idx],
             color_utils::color<cv::Vec3b>(comp_idx));
    } // end for comp_idx
    // paints strings for the faces 3D positions
    for (unsigned int face_idx = 0; face_idx < _faces_centers2d3d.size(); ++face_idx) {
      cv::circle(img_out, _faces_centers2d3d[face_idx].first, 5, CV_RGB(150, 150, 150), -1);
      cv::Point3d face3D = _faces_centers2d3d[face_idx].second;
      std::ostringstream text;
      text << "(" << std::setprecision(2) << face3D.x
           << ", " << face3D.y << ", " << face3D.z << ")";
      cv::putText(img_out, text.str(),
                  _faces_centers2d3d[face_idx].first,
                  CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255));
    }
    cv::imshow("BackgroundSubstractorPPLP_objects", img_out);

    char c = cv::waitKey(25);
    if (c == 's') {
      std::ostringstream prefix;
      prefix << "BackgroundSubstractorPPLP_" << StringUtils::timestamp();
      image_utils::write_rgb_and_depth_image_to_image_file
          (prefix.str(), NULL, &_background);
    }
    else if (c == ' ') {
      printf("BackgroundSubstractorPPLP: storing new background\n");
      depth.copyTo(_background);
    }
  } // end display();

  //////////////////////////////////////////////////////////////////////////////

private:
  //! Canny
  DepthCanny _canny;

  //! disjoint sets
  DisjointSets2 _set;
  std::vector< std::vector<cv::Point> > _components_pts;
  std::vector<cv::Rect> _boundingBoxes;
  std::vector<bool> _comp_was_kept;
  std::vector< std::pair<cv::Point, cv::Point3d> > _faces_centers2d3d;
  int _min_comp_size; // pixels
  std::string _static_frame;
  double _min_comp_z, _max_comp_z;

  cv::Mat1f _background;
  cv::Mat1b _foreground;
  bool _update_background;
  HeadFinder _head_finder;
  cv::Mat1b _user_mask;

  //! the list of faces, with ROS orientation, in RGB frame
  tf::TransformListener* _tf_listener;

  std::vector< cv::Point > _cuts_offsets;
  std::vector<cv::Mat3b> _rgb_cuts;
  std::vector<cv::Mat1f> _depth_cuts;
  std::vector<cv::Mat1b> _user_cuts;

  //! clustering
  image_geometry::PinholeCameraModel _default_depth_camera_model;

  //! information sharing
  people_msgs::PeoplePoseList _ppl;
  ppl_utils::Images2PP _images2pp;

  //! an image for drawing stuff
  cv::Mat3b img_out;
}; // end class BackgroundSubstractorPPLP

#endif // BG_SUBSTRACTOR_PPLP_H
