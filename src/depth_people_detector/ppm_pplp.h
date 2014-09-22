/*!
  \file        ppm_pplp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/7/8

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

Implementation of
"Detecting Pedestrians with Stereo Vision: Safe
Operation of Autonomous Ground Vehicles in
Dynamic Environments"

 */

// utils
#include <src/time/timer.h>
#include <src/geom/Rect3.h>
#include <src/ros_utils/pt_utils.h>
// kinect
#include <src/kinect_utils/kinect_openni_utils.h>
// vision_utils
#include <vision_utils/connected_comp/disjoint_sets2.h>
#include <vision_utils/image_utils/drawing_utils.h>
// people_msgs
#include "src/templates/rgb_depth_pplp_template.h"
#include "src/ppl_utils/images2pp.h"

class Ppm : public RgbDepthPPLPublisherTemplate {
public:
  typedef cv::Point3f Pt3f;
  typedef std::vector<Pt3f> Comp3f;
  typedef std::vector<cv::Point> Comp2i;
  typedef unsigned short CompIdx;

  static const float MIN_USER_WIDTH = .3; // meters - narrow profile
  static const float MAX_USER_WIDTH = 1; // meters - open arms
  static const float MIN_USER_HEIGHT = 1.2; // meters - short person, kid
  static const float MAX_USER_HEIGHT = 2.1; // meters - tall person
  static const float MIN_USER_DEPTH = 0; // meters - flat
  static const float MAX_USER_DEPTH = 1; // meters - open arms, profile

  Ppm() : RgbDepthPPLPublisherTemplate("PPM_PPLP_START", "PPM_PPLP_STOP") {
    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    kinect_openni_utils::read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, rgb_camera_model);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {
    std::vector<geometry_utils::Rect3f> comps_bboxes;
    rois(rgb, depth, _default_depth_camera_model,
         _comps_points, _comps_images, comps_bboxes);
    unsigned int npeople = _comps_images.size();
    // build PPL message
    Timer timer;
    _ppl.header = _images_header; // reuse the header of the last frame
    _ppl.method = "ppm";
    _ppl.poses.resize(npeople);
    for (unsigned int people_idx = 0; people_idx < npeople; ++people_idx) {
      people_msgs::PeoplePose* pp = &(_ppl.poses[people_idx]);
      pp->header = _images_header;
      pp->person_name = people_msgs::PeoplePose::NO_RECOGNITION_MADE;
      pt_utils::copy3(comps_bboxes[people_idx].centroid<Pt3f>(),
                      pp->head_pose.position);
      pp->std_dev = 1; // TODO improve that
      pp->confidence = 1; // TODO improve that
      _images2pp.rgb2user_and_convert(*pp, &_comps_images[people_idx], &depth, true);
    } // end for people_idx
    DEBUG_PRINT("Time for PPL creation:%g ms, %i users\n", timer.time(), npeople);
    publish_PPL(_ppl);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool rois(const cv::Mat & bgr,
            const cv::Mat & depth,
            const image_geometry::PinholeCameraModel & depth_camera_model,
            std::vector<std::vector<Pt3f> > & comps_points,
            std::vector<cv::Mat3b> & comps_images,
            std::vector<geometry_utils::Rect3f> & comps_bboxes) {
    if (!depth_camera_model.initialized()) {
      printf("rois(): depth_cam_model not initialized\n");
      return false;
    }

    Timer timer;
    // convert to Cartesian
    //std::vector<cv::Scalar> colors;
    unsigned int data_step = 2;
    kinect_openni_utils::pixel2world_depth
        (depth, depth_camera_model, _depth_reprojected,
         data_step, cv::Mat(), true);
    unsigned int npts3d = _depth_reprojected.size();
    //  kinect_openni_utils::pixel2world_rgb_color255
    //      (bgr, depth, depth_camera_model, depth_reprojected, colors,
    //       step, cv::Mat(), true);
    _cloud2ppm.resize(npts3d);
    DEBUG_PRINT("Time for pixel2world_depth():%g ms\n", timer.time());

    // directly project each 3D point to 2D PPM image
    float fov = 60 * DEG2RAD; // degrees
    float dist_min = 0, dist_max = 15; // meters
    unsigned int bins_theta = 200, bins_dist = 200;
    float theta_min = M_PI_2 - fov / 2, theta_max = M_PI_2 + fov / 2;
    _ppm.create(bins_dist, bins_theta); // rows, cols - dist in rows, theta in cols
    _ppm.setTo(0);
    assert(_ppm.isContinuous());
    int* ppm_data = (int*) _ppm.ptr();

    for (unsigned int pt_idx = 0; pt_idx < npts3d; ++pt_idx) {
      Pt3f* pt3d = &(_depth_reprojected[pt_idx]);
      int bin_dist = value2bin(bins_dist, dist_min, dist_max,
                               cartesian2ppm_d(pt3d->x, pt3d->z));
      if (bin_dist < 0)
        continue;
      int bin_theta = value2bin(bins_theta, theta_min, theta_max,
                                cartesian2ppm_theta(pt3d->x, pt3d->z));
      if (bin_theta < 0)
        continue;
      // store the index in _cloud2ppm (used in the last step of the algo)
      unsigned int ppm_idx = bin_dist * bins_theta + bin_theta;
      _cloud2ppm[pt_idx] = ppm_idx;

      //++_ppm.at<int>(bin_dist, bin_theta); // at(row, col)
      ++(ppm_data[ppm_idx]);
    } // end for (pt_idx)
    DEBUG_PRINT("Time for ppm creation:%g ms\n", timer.time());

    // threshold ppm
    int ppm_value_thres = 50;
    //  double min, max;
    //  cv::minMaxIdx(ppm, &min, &max);
    //  printf("min:%g, max:%g\n", min, max);
    // cannot use cv::threshold(ppm, ppm_thres) because not same types
    // convert cv::Mat<int> ppm -> cv::Mat<uchar> ppm_thres
    _ppm.convertTo(_ppm_thres, CV_8U);
    cv::threshold(_ppm_thres, _ppm_thres, ppm_value_thres, 255, CV_THRESH_BINARY);
    // make white areas bigger
    cv::dilate(_ppm_thres, _ppm_thres, cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    //cv::morphologyEx(_ppm_thres, _ppm_thres, cv::MORPH_CLOSE,
    //                 cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));

    DEBUG_PRINT("Time for ppm_thres creation:%g ms\n", timer.time());

    // get connect components
    _set.process_image(_ppm_thres);
    DEBUG_PRINT("Time for DisjointSets2 creation:%g ms, nb_comp:%i\n",
               timer.time(), _set.nb_components());

#if 0 // ///////////////////////////////////////////////////////////////////////

    // now build components
    // iterate again on the picture, a bit like pixel2world_rgb_color255()
    int nb_comp = _set.nb_components();
    std::map<DisjointSets2::CompIndex, unsigned int> setindex2vectorindex;
    _comps_points_non_filtered.clear();
    _comps_points_non_filtered.resize(nb_comp);
    _comps_images_non_filtered.clear();
    _comps_points_non_filtered.reserve(nb_comp);
    for (int comp_idx = 0; comp_idx < nb_comp; ++comp_idx)
      _comps_images_non_filtered.push_back(cv::Mat3b(bgr.size(), cv::Vec3b(0, 0, 0)));
    DEBUG_PRINT("Time for comps_images and comps_points allocation:%g ms\n", timer.time());

    // locates matrix header within a parent matrix
    // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
    cv::Size size;
    cv::Point offset;
    bgr.locateROI(size, offset);

    int cols = depth.cols, rows = depth.rows;
    int depth_reprojected_ctr = 0;
    for (int row = 0; row < rows; row += data_step) {
      // get the address of row
      const float* depth_ptr = depth.ptr<float>(row);
      const cv::Vec3b* bgr_data = bgr.ptr<cv::Vec3b>(row);
      for (int col = 0; col < cols; col += data_step) {
        if (std_utils::is_nan_depth(depth_ptr[col])) // NaN depth point
          continue;
        // reproject depth point to 3D - reuse reproject points
        Pt3f* pt3d = &(_depth_reprojected[depth_reprojected_ctr]);

        // get the connected component index
        // get the index stored in _cloud2ppm (used in the last step of the algo)
        unsigned int set_array_idx = _cloud2ppm[depth_reprojected_ctr];
        depth_reprojected_ctr++;

        DisjointSets2::CompIndex set_root_idx = _set.roots[set_array_idx];
        if (set_root_idx == DisjointSets2::NO_NODE) // point not in a comp
          continue;
        // path compression needed as we didn't call _set.get_connected_components()
        set_root_idx = _set.FindSet(set_array_idx);

        // get the corresponding index in the vector
        int vector_idx = -1;
        std::map<DisjointSets2::CompIndex, unsigned int>::iterator it =
            setindex2vectorindex.find(set_root_idx);
        if (it == setindex2vectorindex.end()) { // add the new vector refs
          vector_idx = setindex2vectorindex.size();
          setindex2vectorindex[set_root_idx] = vector_idx;
        }
        else
          vector_idx = it->second;
        if (/*vector_idx < 0 || */ vector_idx >= nb_comp) {
          printf("Error - vector_idx %i out of bounds [%i, %i]!\n",
                 vector_idx, 0, nb_comp);
          continue;
        }

        // now add the point to the vector
        _comps_points_non_filtered[vector_idx].push_back(*pt3d);
        cv::circle(_comps_images_non_filtered[vector_idx], cv::Point(col, row), data_step,
                   cv::Scalar(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]), -1);
        //      cv::rectangle(comps_images[vector_idx],
        //                    cv::Point(col, row),
        //                    cv::Point(col + data_step, row + data_step),
        //                    cv::Scalar(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]), -1);

      } // end loop col
    } // end loop row

    DEBUG_PRINT("Time for component building:%g ms\n", timer.time());

    // now filter the different components according to their bbox
    comps_points.clear();
    comps_points.reserve(_comps_points_non_filtered.size());
    comps_images.clear();
    comps_images.reserve(_comps_images_non_filtered.size());
    comps_bboxes.clear();
    comps_bboxes.reserve(_comps_images_non_filtered.size());

    for (unsigned int comp_idx = 0; comp_idx < _comps_points_non_filtered.size(); ++comp_idx) {
      Comp3f* curr_comp = &(_comps_points_non_filtered[comp_idx]);
      geometry_utils::Rect3f curr_comp_bbox =
          geometry_utils::boundingBox_vec3<float, Pt3f, Comp3f >(*curr_comp);
      if (curr_comp_bbox.height < MIN_USER_HEIGHT
          || curr_comp_bbox.height > MAX_USER_HEIGHT
          || curr_comp_bbox.width < MIN_USER_WIDTH
          || curr_comp_bbox.width > MAX_USER_WIDTH
          || curr_comp_bbox.depth < MIN_USER_DEPTH
          || curr_comp_bbox.depth > MAX_USER_DEPTH) {
        DEBUG_PRINT("comp #%i: comp_bbox '%s' out of bounds\n",
                   comp_idx+1, curr_comp_bbox.to_string().c_str());
        continue;
      }
      DEBUG_PRINT("comp #%i: comp_bbox '%s' in bounds\n", comp_idx+1, curr_comp_bbox.to_string().c_str());
      comps_points.push_back(*curr_comp);
      comps_images.push_back(_comps_images_non_filtered[comp_idx]);
      comps_bboxes.push_back(curr_comp_bbox);
    } // end for comp_idx

#else // ///////////////////////////////////////////////////////////////////////

    // build a conversion matrix ppm2comp
    _set.get_connected_components(_ppm_thres.cols,
                                  _components_pts, _boundingBoxes);
    _ppm2comp.create(_ppm_thres.size());
    _ppm2comp.setTo(0);
    unsigned int ncomps = _boundingBoxes.size(), ppm_cols = _ppm_thres.cols;
    CompIdx* ppm2comp_data = (CompIdx*) _ppm2comp.ptr();
    for (unsigned int comp_idx = 1; comp_idx <= ncomps; ++comp_idx) {
      Comp2i* curr_comp = &(_components_pts[comp_idx-1]);
      unsigned int npts = curr_comp->size();
      for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
        cv::Point pt = (*curr_comp)[pt_idx];
        ppm2comp_data[pt.y * ppm_cols + pt.x] = comp_idx;
      }
    } // end for comp_idx
    DEBUG_PRINT("Time for building ppm2comp:%g ms\n", timer.time());

    // now compute all bounding boxes
    // the default constructor of Rect3f sets width at -1,
    // which ensure proper behaviour for extendToInclude() for the first pt
    std::vector<geometry_utils::Rect3f> _bboxes(ncomps);
    for (unsigned int pt_idx = 0; pt_idx < npts3d; ++pt_idx) {
      unsigned int ppm_idx = _cloud2ppm[pt_idx];
      CompIdx comp_idx = ppm2comp_data[ppm_idx];
      // in _ppm2comp:0 is black, indexing of the comps starts at 1 => substract 1 to comp_idx
      if (comp_idx > 0)
        _bboxes[comp_idx-1].extendToInclude(_depth_reprojected[pt_idx]);
    } // end for pt_idx
    DEBUG_PRINT("Time for computing bboxes:%g ms\n", timer.time());

    // checks on the bounding boxes
    comps_bboxes.clear();
    comps_bboxes.reserve(ncomps);
    std::vector<int> keep_comp2good_idx(ncomps, -1);
    int ngood_comps = 0;
    for (unsigned int bbox_idx = 0; bbox_idx < ncomps; ++bbox_idx) {
      geometry_utils::Rect3f* curr_comp_bbox = &(_bboxes[bbox_idx]);
      if (curr_comp_bbox->height < MIN_USER_HEIGHT
          || curr_comp_bbox->height > MAX_USER_HEIGHT
          || curr_comp_bbox->width < MIN_USER_WIDTH
          || curr_comp_bbox->width > MAX_USER_WIDTH
          || curr_comp_bbox->depth < MIN_USER_DEPTH
          || curr_comp_bbox->depth > MAX_USER_DEPTH) {
        DEBUG_PRINT("comp #%i: comp_bbox '%s' out of bounds\n",
                   bbox_idx+1, curr_comp_bbox->to_string().c_str());
        continue;
      }
      DEBUG_PRINT("comp #%i: comp_bbox '%s' in bounds\n", bbox_idx+1, curr_comp_bbox->to_string().c_str());
      comps_bboxes.push_back(*curr_comp_bbox);
      keep_comp2good_idx[bbox_idx] = ngood_comps;
      ngood_comps++;
    } // end for bbox_idx
    DEBUG_PRINT("Time for bboxes filtering:%g ms, nb_good_comps:%i, bgr:(%i, %i)\n",
               timer.time(), ngood_comps, bgr.cols, bgr.rows);

    // now fetch the comps
    comps_points.clear();
    comps_points.resize(ngood_comps);
    comps_images.resize(ngood_comps);
    for (int comp_idx = 0; comp_idx < ngood_comps; ++comp_idx) {
      comps_images[comp_idx].create(bgr.size());
      comps_images[comp_idx].setTo(0);
    }
    int depth_cols = depth.cols, depth_rows = depth.rows;
    int depth_reprojected_ctr = 0;
    for (int row = 0; row < depth_rows; row += data_step) {
      // get the address of row
      const float* depth_ptr = depth.ptr<float>(row);
      const cv::Vec3b* bgr_data = bgr.ptr<cv::Vec3b>(row);
      for (int col = 0; col < depth_cols; col += data_step) {
        if (std_utils::is_nan_depth(depth_ptr[col])) // NaN depth point
          continue;
        unsigned int ppm_idx = _cloud2ppm[depth_reprojected_ctr];
        CompIdx comp_idx = ppm2comp_data[ppm_idx];
        if (comp_idx == 0) { // not a component
          depth_reprojected_ctr++;
          continue;
        }
        // indexing of the comps in _ppm2comp starts at 1 => substract 1 to comp_idx
        int good_comp_idx = keep_comp2good_idx[comp_idx-1];
        if (good_comp_idx < 0) { // do not keep comp
          depth_reprojected_ctr++;
          continue;
        }

        // reproject depth point to 3D - reuse reproject points
        Pt3f* pt3d = &(_depth_reprojected[depth_reprojected_ctr]);
        depth_reprojected_ctr++;

        comps_points[good_comp_idx].push_back(*pt3d);
        cv::circle(comps_images[good_comp_idx], cv::Point(col, row), data_step,
                   cv::Scalar(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]), -1);
      } // end loop col
    } // end loop row

#endif// ///////////////////////////////////////////////////////////////////////

    DEBUG_PRINT("Time for rois():%g ms, %i comps.\n", timer.time(), comps_bboxes.size());
    return true;
  } // end rois()

  //////////////////////////////////////////////////////////////////////////////

  void display(const cv::Mat3b & rgb, const cv::Mat1f & depth,
               const std::vector<cv::Mat3b> & comps_images) {
    _ppm.convertTo(_ppm_float, CV_32FC1);
    cv::imshow("rgb", rgb);
    cv::imshow("depth", image_utils::depth2viz(depth, image_utils::FULL_RGB_STRETCHED));
    cv::imshow("ppm_float", image_utils::depth2viz(_ppm_float, image_utils::FULL_RGB_STRETCHED, 3));
    cv::imshow("ppm_thres", _ppm_thres);
    cv::imshow("ppm2comp", user_image_to_rgb(_ppm2comp, 16));

    image_utils::imwrite_debug("rgb.png", rgb);
    image_utils::imwrite_debug("depth.png", image_utils::depth2viz(depth, image_utils::FULL_RGB_STRETCHED), image_utils::COLORS256);
    image_utils::imwrite_debug("ppm_float.png", image_utils::depth2viz(_ppm_float, image_utils::FULL_RGB_STRETCHED, 3), image_utils::COLORS256);
    image_utils::imwrite_debug("ppm_thres.png", _ppm_thres, image_utils::MONOCHROME);
    image_utils::imwrite_debug("ppm2comp.png", user_image_to_rgb(_ppm2comp, 16), image_utils::COLORS256);

    if (!_comps_images_non_filtered.empty()) {
      image_utils::paste_images(_comps_images_non_filtered,
                                _comps_images_non_filtered_collage, true, 100, 100, 5,
                                true, titlemaps::int_to_number);
      cv::imshow("comps_images_non_filtered", _comps_images_non_filtered_collage);
      image_utils::imwrite_debug("comps_images_non_filtered.png", _comps_images_non_filtered_collage, image_utils::COLORS256);
    }
    printf("_comps_images.size():%i\n", comps_images.size());
    image_utils::paste_images(comps_images,
                              _comps_images_filtered_collage, true, 100, 100, 5,
                              true, titlemaps::int_to_number);
    cv::imshow("comps_images_filtered", _comps_images_filtered_collage);
    image_utils::imwrite_debug("comps_images_filtered.png", _comps_images_filtered_collage, image_utils::COLORS256);
  }

  virtual void display(const cv::Mat3b & rgb, const cv::Mat1f & depth) {
    display(rgb, depth, _comps_images);
  }

private:

  //////////////////////////////////////////////////////////////////////////////

  /*! \return the discretized bin that corresponds to \arg value
 * in a setting where there are \arg  nbins with range
 * [\arg min, \arg max],
 * or -1 when value is out of range
 */
  static inline int value2bin(const unsigned int nbins,
                              const float & min, const float & max, const float & value) {
    if (value < min || value > max)
      return -1;
    return (value - min) * nbins / (max - min);
  }

  //////////////////////////////////////////////////////////////////////////////

  static inline float cartesian2ppm_d(const float & x, const float & y) {
    return hypot(x, y);
  }

  //////////////////////////////////////////////////////////////////////////////

  static inline float cartesian2ppm_theta(const float & x, const float & y) {
    return atan2(y, x);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the size of the kernel used to try to close contours (pixels)
  static const int MORPH_OPEN_KERNEL_SIZE = 4;

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  /* <scene here>
       ^ z
        \
         +---> x
         |
         |
       y V          */
  Comp3f _depth_reprojected;
  std::vector<unsigned int> _cloud2ppm;
  image_geometry::PinholeCameraModel _default_depth_camera_model;
  cv::Mat1i _ppm;
  cv::Mat1b _ppm_thres;
  DisjointSets2 _set;

  // method1
  std::vector<Comp3f > _comps_points_non_filtered;
  std::vector<cv::Mat3b> _comps_images_non_filtered;

  // method 2
  cv::Mat_<unsigned short> _ppm2comp;
  std::vector< Comp2i > _components_pts;
  std::vector<cv::Rect> _boundingBoxes;

  // conversion to PPL
  std::vector<std::vector<cv::Point3f> > _comps_points;
  std::vector<cv::Mat3b> _comps_images;
  people_msgs::PeoplePoseList _ppl;
  ppl_utils::Images2PP _images2pp;

  // viz
  cv::Mat1f _ppm_float;
  cv::Mat3b _comps_images_non_filtered_collage, _comps_images_filtered_collage;
}; // end Ppm
