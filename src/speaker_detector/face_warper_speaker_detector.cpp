/*!
  \file        reco.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/6/29

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

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/legacy/legacy.hpp"

#include "src/time/timer.h"
#include "src/data_filters/ransac/ransac.h"
#include "src/genetic/genetic.h"
#include "src/geom/flann/affine_transforms.h"
#include "src/ros_utils/pt_utils.h"
#include "vision_utils/image_utils/opencv_face_detector.h"
#include "vision_utils/image_utils/io.h"
// people_msgs
#include "speaker_detector/motion_based_speaker_detector.h"

#define NMETHODS  5 // number of methods
#define METHOD_GOOD_FEATURES 0
#define METHOD_HARRIS_CORNERS 1
#define METHOD_FAST_FEATURES 2
#define METHOD_SURF_AFFINE_TRANSFORM 3
#define METHOD_SURF_HOMOGRAPHY 4

typedef unsigned int Index;

#if 0

class AffineTransformationModel : public ransac::Model<Index>{
public:
  //! store data
  AffineTransformationModel(std::vector<cv::Point2f>* A_,
                            std::vector<cv::Point2f>* B_) : A(A_), B(B_) {}

  /*! \see Model::fit_to_vector()
   * \param data is here the indices of a sample of points of A */
  double fit_to_vector(const DataSet & data) {
    maggieDebug3("fit_to_vector(%s)",
                 StringUtils::accessible_to_string(data).c_str());

    // make sample
    for (unsigned int pt_idx = 0; pt_idx < 3; ++pt_idx) {
      A_sample[pt_idx] = (*A)[data[pt_idx]];
      B_sample[pt_idx] = (*B)[data[pt_idx]];
    } // end loop pt_idx

    cv::getAffineTransform();

    int accumulated = std::accumulate(data.begin(), data.end(), 0);
    mean = (int) (accumulated / data.size());

    // compute error
    double error = 0;
    for (unsigned int data_idx = 0; data_idx < data.size(); ++data_idx)
      error += abs(mean - data[data_idx]);
    error /= data.size();

    maggieDebug3("mean:%i, error:%f", mean, error);
    return error;
  } // end fit_to_vector()

  //! \see Model::distance_to_current()
  double distance_to_current(const int & query) const {
    return abs(mean - query);
  } // end distance_to_current()

  //private:
  std::vector<cv::Point2f>* A;
  std::vector<cv::Point2f>* B;
  cv::Point2f A_sample[3];
  cv::Point2f B_sample[3];
}; // end AffineTransformationModel

////////////////////////////////////////////////////////////////////////////////

//! cf http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/warp_affine/warp_affine.html
class AffineTrans {
  AffineTrans() : scale(1) {}
  cv::Point translation;
  cv::Point center;
  double angle;
  double scale;
};

class AffineTransGeneticSolver : public GeneticSolver<AffineTrans> {
  AffineTransGeneticSolver(const std::vector<cv::Point2f> & A_,
                           const std::vector<cv::Point2f> & B_)
    : A_matA_(A_), B_mat(B_) {
    bboxA = geometry_utils::boundingBox_vec(A_);
    bboxB = geometry_utils::boundingBox_vec(B_);
  }

  inline float fitness(const AffineTrans & to_grade) {
    /// Get the rotation matrix with the specifications above
    rot_mat = cv::getRotationMatrix2D( to_grade.center,
                                       to_grade.angle,
                                       to_grade.scale );
    // aplpy transformation on A to get Atrans
    cv::warpAffine(A_mat, A_mat_trans, rot_mat, A_mat.size());
    // add translation
    for (unsigned int pt_idx = 0; pt_idx < A_mat.cols; ++pt_idx) {
      // p3.at<float>(0, 0) = p.x; // row, col
      A_mat.at<float>(pt_idx, 0) += to_grade.translation.x;
      A_mat.at<float>(pt_idx, 1) += to_grade.translation.y;
    } // end loop pt_idx
    // return distance between Atrans and B
    return cv::norm(A_mat_trans - B_mat, cv::NORM_L1);
  } // end fitness()

  //////////////////////////////////////////////////////////////////////////////

  inline void crossover(const AffineTrans & parent1,
                        const AffineTrans & parent2,
                        AffineTrans & crossed_son) {
    // mix both affine transformations
    crossed_son.center = (parent1.center + parent2.center) / 2;
    crossed_son.angle = (parent1.angle + parent2.angle) / 2;
    crossed_son.scale = (parent1.scale + parent2.scale) / 2;
  } // end crossover();

  //////////////////////////////////////////////////////////////////////////////

  /*! The mutation function.
    \arg mutation_rate
          between 0 (no randomness) and 1 (completely random)
*/
  inline void mutation(const AffineTrans & parent,
                       const float & mutation_rate,
                       AffineTrans & mutated_son) {
    // rotation center: random pt in boxA
    mutated_son.center.x = barycenter(bboxA.x + drand48() * bboxA.width,
                                      parent.center.x,
                                      mutation_rate);
    mutated_son.center.y = barycenter(bboxA.y + drand48() * bboxA.height,
                                      parent.center.y,
                                      mutation_rate);
    mutated_son.angle = barycenter(drand48() * M_PI * 2,
                                   parent.angle,
                                   mutation_rate);
    mutated_son.scale = barycenter(drand48() * 3,
                                   parent.scale,
                                   mutation_rate);
    // translation: random translation from A to B
    unsigned int indA = rand() % A_mat.rows, indB = rand() % B_mat.rows;
    mutated_son.translation.x = barycenter(B_mat.at<float>(indB, 0)
                                           -  A_mat.at<float>(indA, 0),
                                           parent.translation.x,
                                           mutation_rate);
    mutated_son.translation.y = barycenter(B_mat.at<float>(indB, 1)
                                           -  A_mat.at<float>(indA, 1),
                                           parent.translation.y,
                                           mutation_rate);
    // change affine affine
  } // end mutation();

  cv::Rect bboxA, bboxB;
  cv::Mat rot_mat;
  cv::Mat A_mat;
  cv::Mat A_mat_trans;
  cv::Mat B_mat;
}; // end class AffineTransGeneticSolver

#endif

////////////////////////////////////////////////////////////////////////////////

template<class T1, class T2>
void assign_viz_image(const std::vector<T1> & v1,
                      const std::vector<T2> & v2,
                      cv::Mat3b & out) {
  // find bboxes
  cv::Rect bbox1 = geometry_utils::boundingBox_vec< std::vector<T1>, cv::Rect>(v1);
  cv::Rect bbox2 = geometry_utils::boundingBox_vec< std::vector<T2>, cv::Rect>(v2);
  cv::Rect bbox_total = bbox1 | bbox2;
  // find assignment
  CMatrix<assignment_utils::Cost> costs_buffer;
  assignment_utils::MatchList best_assign;
  assignment_utils::Cost best_cost;
  assignment_utils::assign_and_dists
      (v1, v2, costs_buffer, best_assign, best_cost,
       hausdorff_distances::dist_Linf_double
       );
  // paint them
  out.create(bbox_total.y + bbox_total.height,
             bbox_total.x + bbox_total.width); // rows, cols
  out.setTo(0);
  for (unsigned int a_idx = 0; a_idx < best_assign.size(); ++a_idx) {
    if (best_assign[a_idx].first == assignment_utils::UNASSIGNED
        || best_assign[a_idx].second == assignment_utils::UNASSIGNED)
      continue;
    cv::line(out, v1[best_assign[a_idx].first],
        v2[best_assign[a_idx].second],
        CV_RGB(255, 0, 255), 2);
  } // end loop a_idx
}

////////////////////////////////////////////////////////////////////////////////

//! compute NonVerbalActivity of 2 aligned images
void nva(cv::Mat & face1wrapped,
         const cv::Mat & face2) {
  cv::Rect mouth_roi(0, face2.rows / 2, face2.cols, face2.rows / 2);
  cv::Mat buffer;
  NonVerbalActivity act = non_verbal_activity
      (face1wrapped(mouth_roi), face2(mouth_roi), buffer);
  draw_activity(face1wrapped, act);
}

////////////////////////////////////////////////////////////////////////////////

void find_affine_trans_and_wrap_image(const cv::Mat & face1,
                                      const cv::Mat & face2,
                                      const std::vector<cv::Point2f> & corners1,
                                      const std::vector<cv::Point2f> & corners2,
                                      cv::Mat & face1wrapped,
                                      cv::Mat & face1viz,
                                      cv::Mat & face2viz)
{
  cv::Mat3b a1_2_viz;
  assign_viz_image(corners1, corners2, a1_2_viz);
  cv::imshow("a1_2_viz", a1_2_viz);

  face2.copyTo(face1wrapped);
  cv::Mat trans;
  find_affine_transformation_random_samples(cv::Mat(corners1), cv::Mat(corners2),
                                            trans,
                                            //100, true
                                            1000, false
                                            );
  if (!trans.empty()) {
    // keep face2 as a background in case trans is super wrong
    cv::warpAffine(face1, face1wrapped, trans, face2.size(),
                   cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    // apply the trans on the feature points
    //    std::vector<cv::Point2f> corners1_trans;
    //    apply_affine_transformation(corners1, corners1_trans, trans);
    //    cv::Mat3b a1t_2_viz;
    //    assign_viz_image(corners1_trans, corners2, a1t_2_viz);
    //    cv::imshow("a1t_2_viz", a1t_2_viz);
  }

  face1.copyTo(face1viz);
  face2.copyTo(face2viz);
  for (unsigned int corner_idx = 0; corner_idx < corners1.size(); ++corner_idx)
    cv::circle(face1viz, corners1[corner_idx], 3, CV_RGB(255, 255, 0), 2);
  for (unsigned int corner_idx = 0; corner_idx < corners2.size(); ++corner_idx)
    cv::circle(face2viz, corners2[corner_idx], 3, CV_RGB(255, 0, 0), 2);
  cv::imshow("face1viz", face1viz);
  cv::imshow("face2viz", face2viz);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Version of wrap_image() for keypoints.
 * Converts these keypoints into normal 2D points.
 * \param face1
 * \param face2
 * \param keypoints1
 * \param keypoints2
 * \param face1wrapped
 * \param face1viz
 * \param face2viz
 */
void find_affine_trans_and_wrap_image(const cv::Mat & face1,
                                      const cv::Mat & face2,
                                      const std::vector<cv::KeyPoint> & keypoints1,
                                      const std::vector<cv::KeyPoint> & keypoints2,
                                      cv::Mat & face1wrapped,
                                      cv::Mat & face1viz,
                                      cv::Mat & face2viz) {
  // convert keypoints to corners
  std::vector<cv::Point2f> corners1(keypoints1.size());
  for (unsigned int pt_idx = 0; pt_idx < keypoints1.size(); ++pt_idx)
    pt_utils::copy2(keypoints1[pt_idx].pt, corners1[pt_idx]);
  std::vector<cv::Point2f> corners2(keypoints2.size());
  for (unsigned int pt_idx = 0; pt_idx < keypoints2.size(); ++pt_idx)
    pt_utils::copy2(keypoints2[pt_idx].pt, corners2[pt_idx]);
  // call wrap_image();
  find_affine_trans_and_wrap_image(face1, face2, corners1, corners2, face1wrapped,
                                   face1viz, face2viz);
}

////////////////////////////////////////////////////////////////////////////////

void good_features(const cv::Mat & face1,
                   const cv::Mat & face2,
                   cv::Mat & face1viz,
                   cv::Mat & face2viz) {
  cv::Mat face1bw, face2bw;
  cv::cvtColor(face1, face1bw, CV_BGR2GRAY);
  cv::cvtColor(face2, face2bw, CV_BGR2GRAY);
  // Compute good features to track
  std::vector<cv::Point2f> corners1, corners2;
  cv::goodFeaturesToTrack(face1bw,
                          corners1,
                          500, // maximum number of corners to be returned
                          0.01, // quality level
                          10); // minimum allowed distance between points
  cv::goodFeaturesToTrack(face2bw,
                          corners2,
                          500, // maximum number of corners to be returned
                          0.01, // quality level
                          10); // minimum allowed distance between points
  printf("nb good features:%i, %i\n", corners1.size(), corners2.size());
  cv::Mat face1wrapped;
  find_affine_trans_and_wrap_image(face1, face2, corners1, corners2, face1wrapped,
                                   face1viz, face2viz);
  nva(face1wrapped, face2);
  cv::imshow("face1wrapped", face1wrapped);
}

////////////////////////////////////////////////////////////////////////////////

class HarrisDetector {
private:
  // 32-bit float image of corner strength
  cv::Mat cornerStrength;
  // 32-bit float image of thresholded corners
  cv::Mat cornerTh;
  // image of local maxima (internal)
  cv::Mat localMax;
  // size of neighborhood for derivatives smoothing
  int neighbourhood;
  // aperture for gradient computation
  int aperture;
  // Harris parameter
  double k;
  // maximum strength for threshold computation
  double maxStrength;
  // calculated threshold (internal)
  double threshold;
  // size of neighborhood for non-max suppression
  int nonMaxSize;
  // kernel for non-max suppression
  cv::Mat kernel;
public:
  HarrisDetector() : neighbourhood(3), aperture(3),
    k(0.01), maxStrength(0.0),
    threshold(0.01), nonMaxSize(3) {
    // create kernel used in non-maxima suppression
    setLocalMaxWindowSize(nonMaxSize);
  }

  // Create kernel used in non-maxima suppression
  void setLocalMaxWindowSize(int size) {
    nonMaxSize= size;
    kernel.create(nonMaxSize,nonMaxSize,CV_8U);
  }

  // Compute Harris corners
  void detect(const cv::Mat& image) {
    // Harris computation
    cv::cornerHarris(image,cornerStrength,
                     neighbourhood,// neighborhood size
                     aperture,
                     // aperture size
                     k);
    // Harris parameter
    // internal threshold computation
    double minStrength; // not used
    cv::minMaxLoc(cornerStrength,
                  &minStrength,&maxStrength);
    // local maxima detection
    cv::Mat dilated; // temporary image
    cv::dilate(cornerStrength,dilated,cv::Mat());
    cv::compare(cornerStrength,dilated,
                localMax,cv::CMP_EQ);
  }
  // Get the corner map from the computed Harris values
  cv::Mat getCornerMap(double qualityLevel) {
    cv::Mat cornerMap;
    // thresholding the corner strength
    threshold= qualityLevel*maxStrength;
    cv::threshold(cornerStrength,cornerTh,
                  threshold,255,cv::THRESH_BINARY);
    // convert to 8-bit image
    cornerTh.convertTo(cornerMap,CV_8U);
    // non-maxima suppression
    cv::bitwise_and(cornerMap,localMax,cornerMap);
    return cornerMap;
  }
  // Get the feature points from the computed Harris values
  template<class Pt2>
  void getCorners(std::vector<Pt2> &points,
                  double qualityLevel) {
    // Get the corner map
    cv::Mat cornerMap= getCornerMap(qualityLevel);
    // Get the corners
    getCorners(points, cornerMap);
  }

  // Get the feature points from the computed corner map
  template<class Pt2>
  void getCorners(std::vector<Pt2> &points,
                  const cv::Mat& cornerMap) {
    // Iterate over the pixels to obtain all features
    for( int y = 0; y < cornerMap.rows; y++ ) {
      const uchar* rowPtr = cornerMap.ptr<uchar>(y);
      for( int x = 0; x < cornerMap.cols; x++ ) {
        // if it is a feature point
        if (rowPtr[x]) {
          points.push_back(Pt2(x,y));
        }
      }
    }
  }
  // Draw circles at feature point locations on an image
  template<class Pt2>
  void drawOnImage(cv::Mat &image,
                   const std::vector<Pt2> &points,
                   cv::Scalar color= cv::Scalar(0, 255, 0),
                   int radius=3, int thickness=2) {
    typename std::vector<Pt2>::const_iterator it= points.begin();
    // for all corners
    while (it!=points.end()) {
      // draw a circle at each corner location
      cv::circle(image,cv::Point(it->x, it->y),radius,color,thickness);
      ++it;
    }
  }
}; // end class HarrisDetector

void harris_corners(const cv::Mat & face1,
                    const cv::Mat & face2,
                    cv::Mat & face1viz,
                    cv::Mat & face2viz) {
  cv::Mat face1bw, face2bw;
  cv::cvtColor(face1, face1bw, CV_BGR2GRAY);
  cv::cvtColor(face2, face2bw, CV_BGR2GRAY);
  // Create Harris detector instance
  HarrisDetector harris;
  // Compute Harris values
  harris.detect(face1bw);
  std::vector<cv::Point2f> pts1;
  harris.getCorners(pts1,0.01); // get the detected
  harris.detect(face2bw);
  std::vector<cv::Point2f> pts2;
  harris.getCorners(pts2,0.02); // get the detected

  // wrap with found interest points
  cv::Mat face1wrapped;
  find_affine_trans_and_wrap_image(face1, face2, pts1, pts2, face1wrapped, face1viz, face2viz);
  nva(face1wrapped, face2);
  cv::imshow("face1wrapped", face1wrapped);

  // Draw Harris corners
  printf("nb harris corners:%i, %i\n", pts1.size(), pts2.size());
  face1.copyTo(face1viz);
  face2.copyTo(face2viz);
  harris.drawOnImage(face1viz,pts1);
  harris.drawOnImage(face2viz,pts2);
  cv::imshow("face1viz", face1viz);
  cv::imshow("face2viz", face2viz);
}

////////////////////////////////////////////////////////////////////////////////

void fast_features(const cv::Mat & face1,
                   const cv::Mat & face2,
                   cv::Mat & face1viz,
                   cv::Mat & face2viz) {
  // Construction of the Fast feature detector object
  cv::FastFeatureDetector fast(20); // threshold for detection
  // vector of keypoints
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  // feature point detection
  fast.detect(face1,keypoints1);
  fast.detect(face2,keypoints2);

  // wrap with found interest points
  cv::Mat face1wrapped;
  find_affine_trans_and_wrap_image(face1, face2, keypoints1, keypoints2, face1wrapped,
                                   face1viz, face2viz);
  nva(face1wrapped, face2);
  cv::imshow("face1wrapped", face1wrapped);

  printf("nb FAST features:%i, %i\n", keypoints1.size(), keypoints2.size());
  face1.copyTo(face1viz);
  face2.copyTo(face2viz);
  cv::drawKeypoints(face1, // original image
                    keypoints1, // vector of keypoints
                    face1viz, // the output image
                    cv::Scalar(0,255,0), // keypoint color
                    cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //drawing flag
  cv::drawKeypoints(face2,
                    keypoints2,
                    face2viz,
                    cv::Scalar(0,255,0), // keypoint color
                    cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //drawing flag
  cv::imshow("face1viz", face1viz);
  cv::imshow("face2viz", face2viz);
}

////////////////////////////////////////////////////////////////////////////////

cv::Point2f apply_homography(cv::Mat h, cv::Point2f p) {
  cv::Mat p3(3, 1, CV_64F); // rows, cols
  p3.at<double>(0, 0) = p.x; // row, col
  p3.at<double>(1, 0) = p.x; // row, col
  p3.at<double>(2, 0) = 1; // row, col
  cv::Mat ans3 = h * p3;
  return cv::Point2f(ans3.at<double>(0, 0) / ans3.at<double>(2, 0),
                     ans3.at<double>(1, 0) / ans3.at<double>(2, 0));
}

////////////////////////////////////////////////////////////////////////////////

void surf(const cv::Mat & face1,
          const cv::Mat & face2,
          const unsigned short method,
          cv::Mat & face1viz,
          cv::Mat & face2viz) {
  cv::Mat featureface1, featureface2;

  Timer timer;

  // Construct the SURF feature detector object
  cv::SurfFeatureDetector surf(500.); // threshold

  // vector of keypoints
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  // Detect the SURF features
  surf.detect(face1,keypoints1);
  surf.detect(face2,keypoints2);

  timer.printTime("surf keypoint detection");

  // Construction of the SURF descriptor extractor
  cv::SurfDescriptorExtractor surfDesc;
  // Extraction of the SURF descriptors
  cv::Mat descriptors1;
  surfDesc.compute(face1,keypoints1,descriptors1);
  cv::Mat descriptors2;
  surfDesc.compute(face2,keypoints2,descriptors2);

  timer.printTime("SurfDescriptorExtractor");

  // Construction of the matcher
  cv::BruteForceMatcher<cv::L2<float> > matcher;
  // Match the two image descriptors
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1,descriptors2, matches);

  printf("%i matches\n", matches.size());
  //  for (unsigned int match_idx = 0; match_idx < matches.size(); ++match_idx)
  //    printf("%i-%i\n", matches[match_idx].queryIdx, matches[match_idx].trainIdx);

  timer.printTime("BruteForceMatcher");

  if (method == METHOD_SURF_AFFINE_TRANSFORM) {
    unsigned int nkept = 3;
    if (matches.size() > nkept) {
      std::nth_element(matches.begin(),
                       // initial position
                       matches.begin() + nkept, // position of the sorted element
                       matches.end()); // end position
      // remove all elements after
      matches.erase(matches.begin() + nkept, matches.end());
    }
  } // end if (method == METHOD_SURF_AFFINE_TRANSFORM)

  // convert to matching points
  std::vector<cv::Point2f> points1, points2;
  for (unsigned int match_idx = 0; match_idx < matches.size(); ++match_idx) {
    points1.push_back(keypoints1[ matches[match_idx].queryIdx ].pt);
    points2.push_back(keypoints2[ matches[match_idx].trainIdx ].pt);
  }
  //std::cout << "points1:" <<points1 <<std::endl;
  //std::cout << "points2:" <<points2 <<std::endl;

  // Find the homography between image 1 and image 2
  cv::Mat homography;
  std::vector<uchar> inliers(points1.size(),0);

  if (method == METHOD_SURF_AFFINE_TRANSFORM && points1.size() == 3) {
    homography= cv::getAffineTransform(cv::Mat(points1), // corresponding
                                       cv::Mat(points2)); // points
  } else if (points1.size() >= 4)  { // method == METHOD_SURF_HOMOGRAPHY
    homography= cv::findHomography(cv::Mat(points1), // corresponding
                                   cv::Mat(points2), // points
                                   inliers, // outputted inliers matches
                                   CV_RANSAC, // RANSAC method
                                   1.); // max distance to reprojection point
  }

  timer.printTime("findHomography");

  // Warp image 1 to image 2
  cv::Mat face1wrapped;
  face2.copyTo(face1wrapped);
  if (!homography.empty() && method == METHOD_SURF_AFFINE_TRANSFORM) {
    cv::warpAffine(face1,
                   face1wrapped,
                   homography,
                   face2.size(),
                   cv::INTER_LINEAR,
                   cv::BORDER_TRANSPARENT);
  }
  else if (!homography.empty()) { // method == METHOD_SURF_HOMOGRAPHY
    cv::warpPerspective(face1, // input image
                        face1wrapped, // output image
                        homography, // homography
                        face2.size(),
                        cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
  }
  // warp corners
  //  std::cout << "homography:" << homography << std::endl;
  //  std::cout << "c1:" << apply_homography(homography, cv::Point2f(0, 0)) << std::endl;
  //  std::cout << "c2:" << apply_homography(homography, cv::Point2f(0, face1.rows)) << std::endl;
  //  std::cout << "c3:" << apply_homography(homography, cv::Point2f(face1.cols, face1.rows)) << std::endl;
  //  std::cout << "c4:" << apply_homography(homography, cv::Point2f(face1.cols, 0)) << std::endl;
  timer.printTime("warpPerspective");

  // compute NonVerbalActivity
  nva(face1wrapped, face2);

  printf("nb SURF features:%i, %i\n", keypoints1.size(), keypoints2.size());
  // Draw the keypoints with scale and orientation information
  cv::drawKeypoints(face1,
                    // original image
                    keypoints1,
                    // vector of keypoints
                    featureface1,
                    // the resulting image
                    cv::Scalar(0, 255, 0),
                    // color of the points
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //flag
  cv::drawKeypoints(face2,
                    keypoints2,
                    featureface2,
                    cv::Scalar(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //flag
  // Draw the inlier points
  face1.copyTo(face1viz);
  face2.copyTo(face2viz);
  std::vector<cv::Point2f>::const_iterator itPts= points1.begin();
  std::vector<uchar>::const_iterator itIn= inliers.begin();
  while (itPts!=points1.end()) {
    // draw a circle at each inlier location
    if (*itIn)
      cv::circle(face1viz,*itPts,3,
                 cv::Scalar(0, 255, 0),2);
    ++itPts;
    ++itIn;
  }
  itPts= points2.begin();
  itIn= inliers.begin();
  while (itPts!=points2.end()) {
    // draw a circle at each inlier location
    if (*itIn)
      cv::circle(face2viz,*itPts,3,
                 cv::Scalar(0, 255, 0),2);
    ++itPts;
    ++itIn;
  }

  cv::Mat imageMatches;
  cv::drawMatches(face1,keypoints1, // 1st image and its keypoints
                  face2,keypoints2, // 2nd image and its keypoints
                  matches,
                  // the matches
                  imageMatches,
                  // the image produced
                  cv::Scalar(0, 255, 0)); // color of the lines
  timer.printTime("drawMatches");

  cv::imshow("featureface1", featureface1);
  cv::imshow("featureface2", featureface2);
  cv::imshow("imageMatches", imageMatches);
  if (!face1wrapped.empty())
    cv::imshow("face1wrapped", face1wrapped);
}

////////////////////////////////////////////////////////////////////////////////

//! \return false if user exit
bool method_caller(unsigned short & method,
                   const cv::Mat & face1,
                   const cv::Mat & face2,
                   cv::Mat & face1viz,
                   cv::Mat & face2viz) {
  Timer timer;
  if (method == METHOD_GOOD_FEATURES)
    good_features(face1, face2, face1viz, face2viz);
  else if (method == METHOD_HARRIS_CORNERS)
    harris_corners(face1, face2, face1viz, face2viz);
  else if (method == METHOD_FAST_FEATURES)
    fast_features(face1, face2, face1viz, face2viz);
  else if (method == METHOD_SURF_AFFINE_TRANSFORM)
    surf(face1, face2, METHOD_SURF_AFFINE_TRANSFORM, face1viz, face2viz);
  else if (method == METHOD_SURF_HOMOGRAPHY)
    surf(face1, face2, METHOD_SURF_HOMOGRAPHY, face1viz, face2viz);
  timer.printTime("method_caller");

  int c = cv::waitKey(5);
  if (c == 27)
    return false;
  if ((char) c == ' ') {
    method = (method + 1) % NMETHODS;
    printf("method:%i\n", method);
    cv::destroyAllWindows();
  }
  return true;
} // end method_caller();

void test_2_images() {
  cv::Mat face1 = cv::imread("/home/user/face1.png");
  cv::Mat face2 = cv::imread("/home/user/face2.png");
  cv::Mat face1viz, face2viz;
  unsigned short method = 0;
  while(method_caller(method, face1, face2, face1viz, face2viz)) {}
}

////////////////////////////////////////////////////////////////////////////////

void test_camera() {
  cv::VideoCapture capture(0);
  cv::Mat3b frame, prev_frame;
  // create a classifier
  cv::CascadeClassifier classifier = image_utils::create_face_classifier();
  cv::Mat3b small_img;
  std::vector<cv::Rect> curr_rois, prev_rois;
  std::vector<cv::Mat> curr_faces, prev_faces;
  unsigned short method = METHOD_GOOD_FEATURES;
  cv::Mat face1viz, face2viz;

  while (capture.isOpened()) {
    capture >> frame;
    // make the real detection
    Timer timer;
    image_utils::detect_with_opencv(frame, classifier, small_img, curr_rois);
    timer.printTime("detect_with_opencv");
    // shrink rectangles
    //    double ratio_h =  .5;
    //    for (unsigned int rec_idx = 0; rec_idx < curr_rois.size(); ++rec_idx) {
    //      curr_rois[rec_idx].x = curr_rois[rec_idx].x + curr_rois[rec_idx].width * (1. - ratio_h) / 2;
    //      curr_rois[rec_idx].width = curr_rois[rec_idx].width * ratio_h;
    //      curr_rois[rec_idx].height *= .9;
    //    }

    // store faces
    curr_faces.clear();
    for (unsigned int idx = 0; idx < curr_rois.size(); ++idx)
      curr_faces.push_back(frame(curr_rois[idx]).clone()); // clone to change it if needed

    // evaluate user speaking
    if (curr_rois.size() == 1 && prev_rois.size() == 1) {
      bool ok = method_caller(method, prev_faces[0], curr_faces[0], face1viz, face2viz);
      if (!ok)
        break;
    }
    else
      cv::waitKey(5);

    // store data
    frame.copyTo(prev_frame);
    prev_rois = curr_rois;
    prev_faces.clear();
    for (unsigned int face_idx = 0; face_idx < curr_faces.size(); ++face_idx)
      prev_faces.push_back(curr_faces[face_idx]); // no need to clone

    // draw rectangles
    for (unsigned int face_idx = 0; face_idx < curr_rois.size(); ++face_idx)
      cv::rectangle(frame, curr_rois[face_idx], CV_RGB(0, 255, 0), 2);

    // display
    cv::imshow("frame", frame);
    // cv::waitKey() in method_caller()
  } // end while (capture.isOpened)
} // end test_camera();

////////////////////////////////////////////////////////////////////////////////

int main() {
  //test_2_images();
  test_camera();
}
