/*!
  \file        stereo.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/7/19
  
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

http://docs.openorg/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
http://www.neuroforge.co.uk/index.php/77-tutorials/79-stereo-vision-tutorial
 */

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <system/system_utils.h>
#include <ros_utils/pt_utils.h>
#include <string/StringUtils.h>
#include <time/timer.h>

#include <point_clouds/cloud_viewer.h>
#include "bp-vision/stereo_functions.h"

static const bool DEFAULT_FULL_DP = true;
static const int DEFAULT_DISP_12_MAX_DIPP = 13;
static const int DEFAULT_MIN_DISPARITY = 0;
static const int DEFAULT_NUM_DISPARITY = 6 * 16;
static const int DEFAULT_P1 = 172;
static const int DEFAULT_P2 = 1562;
static const int DEFAULT_PRE_FILTER_CAP = 32;
static const int DEFAULT_SAD_WINDOW_SIZE = 3;
static const int DEFAULT_SPECKLE_RANGE = 3;
static const int DEFAULT_SPECKLE_WIN_SIZE = 250;
static const int DEFAULT_UNIQUENESS_RATIO = 10;

////////////////////////////////////////////////////////////////////////////////

void stereoSGBM_gui(const cv::Mat & img1_remapped,
                    const cv::Mat & img2_remapped,
                    cv::Mat & disparity) {
  int FULL_DP = DEFAULT_FULL_DP;
  int DISP_12_MAX_DIPP= DEFAULT_DISP_12_MAX_DIPP;
  int MIN_DISPARITY = DEFAULT_MIN_DISPARITY;
  int NUM_DISPARITY_DIV16 = DEFAULT_NUM_DISPARITY / 16;
  int P1 = DEFAULT_P1;
  int P2 = DEFAULT_P2;
  int PRE_FILTER_CAP = DEFAULT_PRE_FILTER_CAP;
  int SAD_WINDOW_SIZE = DEFAULT_SAD_WINDOW_SIZE;
  int SPECKLE_RANGE = DEFAULT_SPECKLE_RANGE;
  int SPECKLE_WIN_SIZE = DEFAULT_SPECKLE_WIN_SIZE;
  int UNIQUENESS_RATIO = DEFAULT_UNIQUENESS_RATIO;
  std::string window_name = "stereo_gui";
  cv::namedWindow(window_name);
  cv::createTrackbar("FULL_DP(F/T)", window_name, &FULL_DP, 1);
  cv::createTrackbar("DISP_12_MAX_DIPP", window_name, &DISP_12_MAX_DIPP, 50);
  //cv::createTrackbar("MIN_DISqPARITY", window_name, &MIN_DISPARITY, 0);
  cv::createTrackbar("NUM_DISPARITY(X16,>0)", window_name, &NUM_DISPARITY_DIV16, 10);
  cv::createTrackbar("P1", window_name, &P1, 200);
  cv::createTrackbar("P2", window_name, &P2, 2000);
  cv::createTrackbar("PRE_FILTER_CAP", window_name, &PRE_FILTER_CAP, 50);
  cv::createTrackbar("SAD_WINDOW_SIZE(odd)", window_name, &SAD_WINDOW_SIZE, 11);
  cv::createTrackbar("SPECKLE_RANGE", window_name, &SPECKLE_RANGE, 5);
  cv::createTrackbar("SPECKLE_WIN_SIZE", window_name, &SPECKLE_WIN_SIZE, 250);
  cv::createTrackbar("UNIQUENESS_RATIO(5-15)", window_name, &UNIQUENESS_RATIO, 25);

  while (true) {
    printf("'%s':new loop\n", window_name.c_str());
    cv::StereoSGBM stereo(MIN_DISPARITY, NUM_DISPARITY_DIV16 * 16, SAD_WINDOW_SIZE,
                          P1, P2, DISP_12_MAX_DIPP, PRE_FILTER_CAP,
                          UNIQUENESS_RATIO, SPECKLE_WIN_SIZE, SPECKLE_RANGE,
                          FULL_DP);

    Timer timer;
    stereo(img1_remapped, img2_remapped, disparity);
    printf("'%s':stereo succesfully computed in %g ms\n",
           window_name.c_str(), timer.getTimeMilliseconds());
    cv::imshow(window_name, img1_remapped);
    //cv::imshow("disparity", disparity);
    cv::Mat disparity_norm;
    cv::normalize(disparity, disparity_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("disparity_norm", disparity_norm);

    char c = cv::waitKey(0);
    if (c == 'q')
      break;
  } // end while (true)
}

////////////////////////////////////////////////////////////////////////////////

bool read_cal_files(const std::string & cal1filename,
                    const std::string & cal2filename,
                    cv::Mat & cameraMatrix1,
                    cv::Mat & distCoeffs1,
                    cv::Mat & cameraMatrix2,
                    cv::Mat & distCoeffs2,
                    cv::Mat & R,
                    cv::Mat & T) {
  // read file as a big string
  std::string cal1;
  StringUtils::retrieve_file(cal1filename, cal1);
  // put everything as a long line of numbers and remove double spaces
  StringUtils::find_and_replace(cal1, "\n", " ");
  while(StringUtils::find_and_replace(cal1, "  ", " ")) {}
  // convert to a bunch of numerical values
  std::vector<double> values1;
  StringUtils::StringSplit_(cal1, " ", &values1);
  // printf("values1.size():%i\n", values1.size());
  // printf("values1 '%s'\n", StringUtils::accessible_to_string(values1).c_str());
  if (values1.size() != 9 + 4 + 9 + 3) {
    printf("cal1 '%s' ('%s'9 does not contain enough values: %s!\n",
           cal1filename.c_str(), cal1.c_str(), StringUtils::accessible_to_string(values1).c_str());
    return false;
  }
  cameraMatrix1 = (cv::Mat_<double>(3, 3) // First camera matrix.
                   << values1[0], values1[1], values1[2],
      values1[3], values1[4], values1[5],
      values1[6], values1[7], values1[8]);
  distCoeffs1 = (cv::Mat_<double>(4, 1) // First camera distortion parameters.
                 << values1[9], values1[10], values1[11], values1[12]);

  // same thing for file2
  std::string cal2;
  StringUtils::retrieve_file(cal2filename, cal2);
  // put everything as a long line of numbers and remove double spaces
  StringUtils::find_and_replace(cal2, "\n", " ");
  while(StringUtils::find_and_replace(cal2, "  ", " ")) {}
  // convert to a bunch of numerical values
  std::vector<double> values2;
  StringUtils::StringSplit_(cal2, " ", &values2);
  // printf("values2.size():%i\n", values2.size());
  // printf("values2 '%s'\n", StringUtils::accessible_to_string(values2).c_str());
  if (values2.size() != 9 + 4 + 9 + 3) {
    printf("cal2 '%s' ('%s'9 does not contain enough values: %s!\n",
           cal2filename.c_str(), cal2.c_str(), StringUtils::accessible_to_string(values2).c_str());
    return false;
  }
  cameraMatrix2 = (cv::Mat_<double>(3, 3) // First camera matrix.
                   << values2[0], values2[1], values2[2],
      values2[3], values2[4], values2[5],
      values2[6], values2[7], values2[8]);
  distCoeffs2 = (cv::Mat_<double>(4, 1) // First camera distortion parameters.
                 << values2[9], values2[10], values2[11], values2[12]);

  // read R, T
  R = (cv::Mat_<double>(3, 3) //  Rotation matrix between the coordinate systems of the first and the second cameras.
       << values2[13], values2[14], values2[15],
      values2[16], values2[17], values2[18],
      values2[19], values2[20], values2[21]);
  T = (cv::Mat_<double>(3, 1) //  Translation vector between coordinate systems of the cameras.
       << values2[22], values2[23], values2[24]);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief init_rectif_maps
 * \param cameraMatrix1
 * \param distCoeffs1
 * \param cameraMatrix2
 * \param distCoeffs2
 * \param R
 * \param T
 * \param map1x
 * \param map1y
 * \param map2x
 * \param map2y
 * \param Q
 *    will be useful in reproject();
 */
void init_rectif_maps(const cv::Mat & cameraMatrix1,
                      const cv::Mat & distCoeffs1,
                      const cv::Mat & cameraMatrix2,
                      const cv::Mat & distCoeffs2,
                      const cv::Mat & R,
                      const cv::Mat & T,
                      const cv::Size & imageSize,
                      cv::Mat & map1x,
                      cv::Mat & map1y,
                      cv::Mat & map2x,
                      cv::Mat & map2y,
                      cv::Mat & Q) {
  //R1 – Output 3x3 rectification transform (rotation matrix) for the first camera.
  //R2 – Output 3x3 rectification transform (rotation matrix) for the second camera.
  //P1 – Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.
  //P2 – Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.
  //Q – Output  4x4 disparity-to-depth mapping matrix (see reprojectImageTo3D() ).
  cv::Mat R1, R2, P1, P2;

  // alpha – Free scaling parameter.
  // If it is -1 or absent, the function performs the default scaling.
  // Otherwise, the parameter should be between 0 and 1.
  // alpha=0 means that the rectified images are zoomed and shifted so that
  // only valid pixels are visible (no black areas after rectification).
  // alpha=1 means that the rectified image is decimated and shifted so that
  // all the pixels from the original images from the cameras are retained
  // in the rectified images (no source image pixels are lost).
  // Obviously, any intermediate value yields an intermediate result between
  // those two extreme cases.
  cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2,distCoeffs2, imageSize,
                    R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 0);

  /* rectify images
   */
  // R – Optional rectification transformation in the object space (3x3 matrix). R1 or R2 , computed by stereoRectify()
  // In case of a stereo camera, newCameraMatrix is normally set to P1 or P2 computed by stereoRectify()
  // m1type – Type of the first output map that can be CV_32FC1 or CV_16SC2
  cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1,
                              imageSize, CV_32FC1, map1x, map1y);
  cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2,
                              imageSize, CV_32FC1, map2x, map2y);
}

////////////////////////////////////////////////////////////////////////////////

enum DisparityAlgorithm {
  DISPARITY_BELIEF_PROP = 0,
  DISPARITY_StereoSGBM = 1,
  DISPARITY_StereoSGBM_GUI = 2,
  DISPARITY_StereoBM = 3,
};

/*!
 * \brief compute_disparity
 * \param img1, img2
 *    Input images, not rectified.
 * \param map1x, map1y, map2x, map2y
 *    Rectification maps obtained with \a init_rectif_maps()
 * \param img1_remapped, img2_remapped
 *    Some buffers where \a img1 and \a img2 will be remapped.
 * \param disparity
 *    Output disparity
 */
void compute_disparity(const cv::Mat & img1,
                       const cv::Mat & img2,
                       const cv::Mat & map1x,
                       const cv::Mat & map1y,
                       const cv::Mat & map2x,
                       const cv::Mat & map2y,
                       cv::Mat & img1_remapped,
                       cv::Mat & img2_remapped,
                       cv::Mat & disparity,
                       DisparityAlgorithm algo = DISPARITY_StereoSGBM) {
  cv::remap(img1, img1_remapped, map1x, map1y, cv::INTER_LINEAR);
  cv::remap(img2, img2_remapped, map2x, map2y, cv::INTER_LINEAR);

  // compute disparity
  if (algo == DISPARITY_BELIEF_PROP) {
#if 0 // extern call to disparity_binary
    // nedds B&W conv
    cv::Mat img1_remapped_bw;
    cv::cvtColor(img1_remapped, img1_remapped_bw, CV_BGR2GRAY);
    cv::Mat img2_remapped_bw;
    cv::cvtColor(img2_remapped, img2_remapped_bw, CV_BGR2GRAY);
    std::string filename1_pgm = "/tmp/f1.pgm";
    cv::imwrite(filename1_pgm, img1_remapped_bw);
    std::string filename2_pgm = "/tmp/f2.pgm";
    cv::imwrite(filename2_pgm, img2_remapped_bw);

    std::string disparity_binary = "/home/user/Downloads/0datasets/bp-vision/stereo";
    std::string disparity_filename = "/tmp/disparity.pgm";
    std::ostringstream order;
    order << disparity_binary
          << " " << filename1_pgm
          << " " << filename2_pgm
          << " " << disparity_filename;
    system_utils::exec_system(order.str());
    cv::Mat disparity255 = cv::imread(disparity_filename, CV_LOAD_IMAGE_GRAYSCALE);
    // as scale from stereo.cpp is 16, and disparity maps in OpenCV
    // have the same scale, no conversion needed
    disparity255.convertTo(disparity, CV_16S);
#else // C++ method to BP
    // nedds B&W conv
    cv::Mat img1_remapped_bw;
    cv::cvtColor(img1_remapped, img1_remapped_bw, CV_BGR2GRAY);
    cv::Mat img2_remapped_bw;
    cv::cvtColor(img2_remapped, img2_remapped_bw, CV_BGR2GRAY);

    // BP format
    int data1_len = img1.cols * img1.rows * sizeof(uchar);
    image<uchar> img1_bp(img1.cols, img1.rows); // width, height
    memcpy(img1_bp.data, img1_remapped_bw.data, data1_len);
    int data2_len = img2.cols * img2.rows * sizeof(uchar);
    image<uchar> img2_bp(img2.cols, img2.rows);
    memcpy(img2_bp.data, img2_remapped_bw.data, data2_len);

    // BP
    image<uchar>* out = stereo_ms(&img1_bp, &img2_bp);

    // copy back to to OpenCV
    disparity.create(img1.rows, img1.cols, CV_8U); // rows, cols
    memcpy(disparity.data, out->data, data1_len);
    // clean
    delete out;
#endif
  } // end if (algo == DISPARITY_BELIEF_PROP)

  else if (algo == DISPARITY_StereoSGBM) { // cv::StereoSGBM - params obtained from GUI
    cv::StereoSGBM stereo
        (DEFAULT_MIN_DISPARITY, DEFAULT_NUM_DISPARITY, DEFAULT_SAD_WINDOW_SIZE,
         DEFAULT_P1, DEFAULT_P2, DEFAULT_DISP_12_MAX_DIPP, DEFAULT_PRE_FILTER_CAP,
         DEFAULT_UNIQUENESS_RATIO, DEFAULT_SPECKLE_WIN_SIZE, DEFAULT_SPECKLE_RANGE,
         DEFAULT_FULL_DP);

    stereo(img1_remapped, img2_remapped, disparity);
  } // end if (algo == DISPARITY_StereoSGBM)

  else if (algo == DISPARITY_StereoSGBM_GUI) {
    stereoSGBM_gui(img1_remapped, img2_remapped, disparity);
  } // end if (algo == DISPARITY_StereoSGBM_GUI)

  else if (algo == DISPARITY_StereoBM) {
    // nedds B&W conv
    cv::Mat img1_remapped_bw;
    cv::cvtColor(img1_remapped, img1_remapped_bw, CV_BGR2GRAY);
    cv::Mat img2_remapped_bw;
    cv::cvtColor(img2_remapped, img2_remapped_bw, CV_BGR2GRAY);
    // values from PDF
    int preset=cv::StereoBM::BASIC_PRESET;
    int ndisparities=128;
    int SADWindowSize=41;
    cv::StereoBM stereo(preset, ndisparities, SADWindowSize);
    stereo(img1_remapped_bw, img2_remapped_bw, disparity);
  } // end if (algo == DISPARITY_StereoBM)
}

////////////////////////////////////////////////////////////////////////////////

void reproject_disparity_to_3D_point_cloud(const cv::Mat & img1_remapped,
                                           const cv::Mat & disparity,
                                           const cv::Mat & Q,
                                           cv::Mat & _3dImage,
                                           std::vector<cv::Point3f> & pointcloud,
                                           std::vector<cv::Vec3b> & pointcloud_RGB) {
  // reproject to 3D
  // _3dImage – Output 3-channel floating-point image of the same size as disparity .
  // Each element of _3dImage(x,y) contains 3D coordinates of the point (x,y)
  // computed from the disparity map.
  cv::reprojectImageTo3D(disparity, _3dImage, Q, false, CV_32F);
  int npts = _3dImage.rows * _3dImage.cols;
  pointcloud.clear();
  pointcloud.reserve(npts);
  pointcloud_RGB.clear();
  pointcloud_RGB.reserve(npts);
  bool is_calibration_in_meters = (fabs(Q.at<double>(3, 2)) > .1); // at(row,col)
  std::cout << "Q:" << Q << std::endl;
  printf("Q(0, 3):%g\n", Q.at<double>(0, 3));

  for (int row = 0; row < _3dImage.rows; ++row) {
    // get the address of row
    const cv::Point3f* _3dImage_data = _3dImage.ptr<cv::Point3f>(row);
    const cv::Vec3b* img1_data = img1_remapped.ptr<cv::Vec3b>(row);
    for (int col = 0; col < _3dImage.cols; ++col) {
      // remove NaN points
      if (_3dImage_data[col].x != _3dImage_data[col].x || isnan(_3dImage_data[col].x) ||
          _3dImage_data[col].y != _3dImage_data[col].y || isnan(_3dImage_data[col].y) ||
          _3dImage_data[col].z != _3dImage_data[col].z || isnan(_3dImage_data[col].z) ||
          (img1_data[col][0] || img1_data[col][1] || img1_data[col][2]) == 0)
        continue;

      if (rand() % 10000 == 0) {
        printf("pt3D:%s, color:(%i.%i,%i)\n",
               pt_utils::print_point(_3dImage_data[col]).c_str(),
               img1_data[col][0], img1_data[col][1], img1_data[col][2]);
      }
      if (is_calibration_in_meters)
        pointcloud.push_back(_3dImage_data[col]);
      else
        pointcloud.push_back(1/1000.f * _3dImage_data[col]);
      pointcloud_RGB.push_back(img1_data[col]);
    } // end loop col
  } // end loop row
} // end reproject_disparity_to_3D_point_cloud()

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief test
 * \param filename1
 * \param filename2
 * \param calfilename1
 * \param calfilename2
 * \param imageSize
 *  Size of the image used for stereo calibration.
 * \return
 */
int test_color_image(std::string filename1,
                     std::string filename2,
                     std::string calfilename1,
                     std::string calfilename2,
                     DisparityAlgorithm algo = DISPARITY_StereoSGBM) {
  printf("test(files '%s', '%s', calibration files '%s', '%s', algo:%i)\n",
         filename1.c_str(), filename2.c_str(),
         calfilename1.c_str(), calfilename2.c_str(), algo);
  Timer timer;
  cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T;
  read_cal_files(calfilename1, calfilename2,
                 cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T);

  cv::Mat map1x, map1y, map2x, map2y, Q;
  cv::Mat img1 = cv::imread(filename1), img2 = cv::imread(filename2);
  cv::Size imageSize = img1.size();
  init_rectif_maps(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, imageSize,
                   map1x, map1y, map2x, map2y, Q);

  cv::Mat img1_remapped, img2_remapped, disparity;
  compute_disparity(img1, img2, map1x, map1y, map2x, map2y,
                    img1_remapped, img2_remapped, disparity, algo);

  cv::Mat _3dImage;
  std::vector<cv::Point3f> pointcloud;
  std::vector<cv::Vec3b> pointcloud_RGB;
  reproject_disparity_to_3D_point_cloud(img1_remapped, disparity, Q,
                                        _3dImage, pointcloud, pointcloud_RGB);

  timer.printTime("time for everything");

  double minVal, maxVal;
  cv::minMaxIdx(disparity, &minVal, &maxVal);
  printf("minVal:%g, maxVal:%g\n", minVal, maxVal);

  cv::imshow("img1_remapped", img1_remapped);
  cv::imshow("img2_remapped", img2_remapped);
  //cv::imshow("disparity", disparity);
  cv::Mat disparity_norm;
  cv::normalize(disparity, disparity_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::imshow("disparity_norm", disparity_norm);
  cv::imwrite("img1_remapped.png", img1_remapped);
  cv::imwrite("img2_remapped.png", img2_remapped);
  cv::waitKey(0);
  cloud_viewer::view_rgb_cloud(pointcloud, pointcloud_RGB);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if (argc == 5) { // 2 files, 2 calib files
    test_color_image(argv[1], argv[2], argv[3], argv[4]);
    return 0;
  }
  if ( argc == 6) { // 2 files, 2 calib files, algo
    test_color_image(argv[1], argv[2], argv[3], argv[4],
        static_cast<DisparityAlgorithm>(atoi(argv[5])));
    return 0;
  }
  //    test("/home/user/Downloads/0datasets/car/image_00004900_0.png",
  //         "/home/user/Downloads/0datasets/car/image_00004900_1.png",
  //         "/home/user/Downloads/0datasets/car/cam1.cal",
  //         "/home/user/Downloads/0datasets/car/cam2.cal");

  //  test("/home/user/Downloads/0datasets/sunny/image_00000100_0.png",
  //       "/home/user/Downloads/0datasets/sunny/image_00000100_1.png",
  //       "/home/user/Downloads/0datasets/sunny/cam1.cal",
  //       "/home/user/Downloads/0datasets/sunny/cam2.cal");

  test_color_image("LINTHESCHER/image_00000930_0.png",
                   "LINTHESCHER/image_00000930_1.png",
                   "LINTHESCHER/cam1.cal",
                   "LINTHESCHER/cam2.cal");
  return 0;
}
