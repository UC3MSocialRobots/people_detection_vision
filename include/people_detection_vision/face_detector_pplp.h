/*!
  \file        face_detector_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/24

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

\class FaceDetectorPPLP
A face detector for RGBD images.
It consists of two steps:
  1- "classic" face detection.
     Using OpenCV implementation of Viola and Jones face detection algorithm,
     faces are found in the color image
  2- false positive removal
     For each found face, a set of 3D points inside the bounding box of this
     face is computed. The 3D reprojection is possible thanks to the depth image.
     The 3D bounding box of the set is then evaluated.
     If the dimensions of this 3D bounding box do not match given criteria
     (especially width and height), the face is classified as false positive
     and discarded.

\section Parameters
  - \b depth_topic_name
    [std::string] (default: "depth")
    where to get the depth images

  - \b rgb_topic
    [std::string] (default: "rgb")
    where to get the rgb images

\section Subscriptions
  - \b {depth_topic_name}
    [sensor_msgs/Image]
    The depth image

  - \b {rgb_topic}
    [sensor_msgs/Image]
    The rgb image

  - \b {camera_info_topic}
    [sensor_msgs/CameraInfo]
    The model of the camera giving the depth images

\section Publications
  - \b "~ppl"
        [people_msgs::People]
        The found people heads

*/

#ifndef FACE_DETECTOR_PPLP_PPL_H
#define FACE_DETECTOR_PPLP_PPL_H

// dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <people_detection_vision/ViolaConfig.h>
// AD
#include "vision_utils/Rect3.h"
#include "vision_utils/timer.h"

#include "vision_utils/opencv_face_detector.h"
#include "vision_utils/blob_segmenter.h"

// people_msgs
#include "vision_utils/rgb_depth_pplp_template.h"
#include "vision_utils/images2ppl.h"

class FaceDetectorPPLP : public RgbDepthPPLPublisherTemplate {
public:
  typedef cv::Point3d Pt3d;

  // false positive discriminaton
  //! the number of depth samples
  static const unsigned int DEPTH_SAMPLE_SIZE = 10;
  //! the radius of a head, from the eye surface to the head center (m)
  static const double HEAD_DEPTH = 0.15;
  //! from forehead to chin / 2
  static const double HEAD_HEIGHT = 0.25;
  static const int MAX_SAMPLE_TRIES = 20 * DEPTH_SAMPLE_SIZE;

  FaceDetectorPPLP() :
    RgbDepthPPLPublisherTemplate("FACE_DETECTOR_PPLP_START", "FACE_DETECTOR_PPLP_STOP")
  {
    _cascade_classifier = vision_utils::create_face_classifier();

    // params
    //! the maximum bounding-box depth of the face points (m)
    _nh_private.param("max_sample_depth", _max_sample_depth, 0.15);
    //! the maximum bounding-box width of the face points (m)
    _nh_private.param("max_sample_width", _max_sample_width, 0.2);

    // 320 240 1.2 1 10: 1424 ms
    // 320 240 1.3 0 5: 1044 ms
    _nh_private.param("resize_max_width", _resize_max_width, 320);
    _nh_private.param("resize_max_height", _resize_max_height, 240);
    _nh_private.param("scale_factor", _scale_factor, 1.2);
    _nh_private.param("min_neighbors", _min_neighbors, 1);
    _nh_private.param("min_width", _min_width, 10);
    f = boost::bind(&FaceDetectorPPLP::config_callback, this, _1, _2);
    server.setCallback(f);

    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    vision_utils::read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, rgb_camera_model);
    printf("FaceDetectorPPLP: getting rgb on '%s', depth on '%s', "
           "publish People results on '%s', _display:%i\n",
           get_rgb_topic().c_str(), get_depth_topic().c_str(),
           get_ppl_topic().c_str(), _display);
  }

  ///////////////////////////////////////////////////////////////////////////////

  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {
    vision_utils::Timer timer;
    // clear data
    _faces_filtered.clear();
    _faces_centers_3d.clear();
    _users.clear();

    // detect with opencv
    vision_utils::detect_with_opencv
        (rgb, _cascade_classifier,
         _small_img, _faces_not_filtered,
         _resize_max_width, _resize_max_height, _scale_factor,
         _min_neighbors, _min_width);
    DEBUG_PRINT("time for detect_with_opencv(): %g ms, %i faces\n",
                timer.time(), _faces_not_filtered.size());

    // remove including faces
    std::vector< cv::Rect > faces_not_filtered_orig = _faces_not_filtered;
    vision_utils::remove_including_rectangles
        (faces_not_filtered_orig, _faces_not_filtered);
    DEBUG_PRINT("time for remove_including_rectangles(): %g ms, %i faces\n",
                timer.time(), _faces_not_filtered.size());

    filter_outliers_with_depth(depth);
    DEBUG_PRINT("Time for filter_outliers_with_depth(): %g ms, "
                "after filtering, %i faces: %s\n",
                timer.time(), _faces_filtered.size(),
                vision_utils::accessible_to_string(_faces_centers_3d).c_str());
    if (get_ppl_num_subscribers() > 0)
      build_ppl_message(rgb, depth);
    _last_time = timer.time();
    DEBUG_PRINT("Time for process_rgb_depth():%g m, %i users\n", _last_time, nusers);

    if (_display)
      display(rgb, depth);
  } // end process_rgb_depth();

  //////////////////////////////////////////////////////////////////////////////

  /*! share the poses of the detected users in a
   *  people_msgs::People msg */
  void build_ppl_message(const cv::Mat3b & rgb,
                         const cv::Mat1f & depth) {
    // printf("build_ppl_message()\n");
    // share the poses
    unsigned int n_faces = _faces_centers_3d.size();
    _ppl.header = _images_header;
    _ppl.people.resize(n_faces);


    // build the Person
    for (unsigned int user_idx = 0; user_idx < n_faces; ++user_idx) {
      people_msgs::Person* pp = &(_ppl.people[user_idx]);
      vision_utils::set_method(*pp, "face_detector");
      pp->header = _ppl.header; // copy header
      // people_pose.name = vision_utils::cast_to_string(user_idx);
      pp->name = "NOREC";
      pp->reliability = 1;

      // pose
      vision_utils::copy3(_faces_centers_3d[user_idx], pp->position);
      pp->position.orientation = tf::createQuaternionMsgFromYaw(0);

      // image
      if (!_images2pp.convert(*pp, &rgb, &depth, &(_users[user_idx]), true))
        continue;
    } // end loop user_idx

    publish_PPL(_ppl);
  } // end build_ppl_message()

  //////////////////////////////////////////////////////////////////////////////

  void filter_outliers_with_depth(const cv::Mat1f & depth) {
    // DEBUG_PRINT("filter_outliers_with_depth()\n");
    _faces_filtered.reserve(_faces_not_filtered.size());
    _faces_centers_3d.reserve(_faces_not_filtered.size());

    for (unsigned int user_idx = 0; user_idx < _faces_not_filtered.size(); ++user_idx) {
      // get a small rect centered on the face
      cv::Rect curr_face = _faces_not_filtered.at(user_idx);
      cv::Rect curr_face_shrunk = vision_utils::shrink_rec(curr_face, 0.5);

      // check if seed already seen in user mask
      cv::Point user_mask_seed = .5 * (curr_face.tl() + curr_face.br());
      for (unsigned int prev_user_idx = 0; prev_user_idx < _users.size(); ++prev_user_idx) {
        if (vision_utils::bbox_full(_users[prev_user_idx]).contains(user_mask_seed)
            && _users[prev_user_idx](user_mask_seed) > 0) {
          DEBUG_PRINT("Face %i: blob at seed (%i, %i) already seen before \n",
                      prev_user_idx, user_mask_seed.x, user_mask_seed.y);
          continue;
        }
      } // end for prev_user_idx

      // get a sample of 3D points located in it
      std::vector<Pt3d> face_sample;
      int total_samples = 0;
      while (face_sample.size() < DEPTH_SAMPLE_SIZE && total_samples < MAX_SAMPLE_TRIES) {
        ++total_samples;
        // get a 2D random point
        cv::Point curr_face_pt2d (curr_face_shrunk.x + rand() % curr_face_shrunk.width,
                                  curr_face_shrunk.y + rand() % curr_face_shrunk.height);
        // reproject it
        Pt3d curr_face_pt3d = //pixel2world_rgb(curr_face_pt2d);
            vision_utils::pixel2world_depth<Pt3d>
            (curr_face_pt2d, _default_depth_camera_model,
             depth);
        // dismiss the NaN points
        if (isnan(curr_face_pt3d.x) || isnan(curr_face_pt3d.y) || isnan(curr_face_pt3d.z))
          continue;
        // keep this valid point
        face_sample.push_back(curr_face_pt3d);
      } // end point collecting

      if (face_sample.size() == 0) {
        printf("Face %i: We could not find any sample points for that face !\n",
               user_idx);
        continue;
      }

      // now compute the bounding box of this sample
      vision_utils::Rect3d bbox =
          vision_utils::boundingBox_vec3
          <double, Pt3d, std::vector<Pt3d> >(face_sample);
      // printf("Face %i: bbox:%s\n", user_idx, bbox.to_string().c_str());

      // tests on the geometry of the bounding box
      if (bbox.depth > _max_sample_depth) {
        DEBUG_PRINT("Face %i: bbox.depth=%g > _max_sample_depth=%g: skipping\n",
                    user_idx, bbox.depth, _max_sample_depth);
        continue;
      }
      if (bbox.width > _max_sample_width) {
        DEBUG_PRINT("Face %i: bbox.width=%g > _max_sample_width=%g: skipping\n",
                    user_idx, bbox.width, _max_sample_width);
        continue;
      }

      // last test (slow)
      cv::Mat1b curr_user_mask;
      if (!_segmenter.find_blob(depth, user_mask_seed, curr_user_mask,
                                BlobSegmenter::GROUND_PLANE_FINDER,
                                NULL,
                                true,
                                DepthCanny::DEFAULT_CANNY_THRES1,
                                DepthCanny::DEFAULT_CANNY_THRES2,
                                GroundPlaneFinder::DEFAULT_DISTANCE_THRESHOLD_M,
                                GroundPlaneFinder::DEFAULT_LOWER_RATIO_TO_USE,
                                4)) // data skip
        continue; // cant find blob

      // we passed all tests : keep the face
      _users.push_back(curr_user_mask.clone());

      _faces_filtered.push_back(curr_face);
      Pt3d face_center_3d_ros_orien = bbox.centroid<Pt3d>();
      face_center_3d_ros_orien.z += HEAD_DEPTH;
      // add the radius of the head
      _faces_centers_3d.push_back(face_center_3d_ros_orien);
    } // end loop user_idx
  } // end filter_outliers_with_depth();

  //////////////////////////////////////////////////////////////////////////////

  virtual void display(const cv::Mat3b & rgb,
                       const cv::Mat1f & depth) {
    rgb.copyTo(_img_out);
    std::ostringstream txt; txt << (int) _last_time << " ms";
    cv::putText(_img_out, txt.str(), cv::Point(10, 30),
                CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(0, 0, 255));
    // convert the depth to an RGB image for display
    //    vision_utils::convert_float_to_uchar(depth, depth_color_to_uchar,
    //                                        alpha_trans, beta_trans);
    //    cv::imshow("depth_color_to_uchar", depth_color_to_uchar);

    // paint each user mask in _img_out
    for (unsigned int user_idx = 0; user_idx < _users.size(); ++user_idx) {
      cv::Vec3b color = vision_utils::color<cv::Vec3b>(user_idx);
      color += cv::Vec3b(70, 70, 70); //more pale colors
      _img_out.setTo(color, _users[user_idx]);
      //cv::add(_img_out, cv::Scalar::all(150), _img_out, _users[user_idx]);
    }

    // removed faces in red
    for (unsigned int user_idx = 0; user_idx < _faces_not_filtered.size(); ++user_idx) {
      cv::Rect curr_face = _faces_not_filtered.at(user_idx);
      cv::rectangle(_img_out, curr_face, CV_RGB(255, 0, 0), 2);
      // get a small rect centered on the face
      cv::Rect curr_face_shrunk = vision_utils::shrink_rec(curr_face, 0.5);
      cv::rectangle(_img_out, curr_face_shrunk, CV_RGB(255, 255, 0), 2);
    }
    // kept faces in green
    for (unsigned int user_idx = 0; user_idx < _faces_filtered.size(); ++user_idx)
      cv::rectangle(_img_out, _faces_filtered[user_idx], CV_RGB(0, 255, 0), 2);

    //  if (!_users.empty()) {
    //    cv::Mat1b users_collage;
    //    vision_utils::paste_images(_users, users_collage, true, 100, 100);
    //    cv::imshow("users_collage", users_collage);
    //  }

    //cv::imshow("rgb", rgb);
    //cv::imshow("depth", vision_utils::depth2viz(depth, vision_utils::FULL_RGB_STRETCHED));
    cv::imshow("FaceDetectorPPLP", _img_out);
    int key_code = (char) cv::waitKey(1);
    if (key_code == 27)
      ros::shutdown();
  }

  //////////////////////////////////////////////////////////////////////////////


private:

  void config_callback(people_detection_vision::ViolaConfig &config, uint32_t level) {
    ROS_INFO("config_callback()");
    _resize_max_width = config.resize_max_width;
    _resize_max_height = config.resize_max_height;
    _scale_factor = config.scale_factor;
    _min_neighbors = config.min_neighbors;
    _min_width = config.min_width;
    _max_sample_depth = config.max_sample_depth;
    _max_sample_width = config.max_sample_width;
  }

  /* face detection */
  //! the redim image
  cv::Mat3b _small_img;
  //! the list of faces found
  std::vector< cv::Rect > _faces_not_filtered;
  //! the classifier
  cv::CascadeClassifier _cascade_classifier;
  int _resize_max_width, _resize_max_height, _min_neighbors, _min_width;
  double _scale_factor;

  /* reprojection and filtering */
  double _max_sample_depth, _max_sample_width;
  image_geometry::PinholeCameraModel _default_depth_camera_model;
  BlobSegmenter _segmenter;

  // shared data! the three vectors have the same size
  //! the filtered list of faces found
  std::vector< cv::Rect > _faces_filtered;
  //! the list of faces, with ROS orientation, in RGB frame
  std::vector< Pt3d > _faces_centers_3d;
  //! the mask computed for each user
  std::vector<cv::Mat1b> _users;

  /* visu */
  //! an image for drawing stuff
  cv::Mat3b _img_out;
  double _last_time;

  // PPL
  people_msgs::People _ppl;
  vision_utils::Images2PP _images2pp;

  dynamic_reconfigure::Server<people_detection_vision::ViolaConfig> server;
  dynamic_reconfigure::Server<people_detection_vision::ViolaConfig>::CallbackType f;
}; // end class FaceDetectorPPLP

#endif // FACE_DETECTOR_PPLP_PPL_H
