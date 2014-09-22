/*!
  \file        color_face_detector.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/6
  
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

#ifndef COLOR_FACE_DETECTOR_H
#define COLOR_FACE_DETECTOR_H

#include <opencv2/highgui/highgui.hpp>
// vision
#include <vision_utils/visu_utils/array_to_color.h>
//#include <vision_utils/image_utils/io.h>
// utils
#include <src/gnuplot-cpp/gnuplot_i.hpp>
#include <src/system/system_utils.h>
#include <src/stats/stats_utils.h>

#include <vision_utils/image_utils/opencv_face_detector.h>

//#define LUT_ELEM_TYPE float
//#define LUT_ELEM_TYPE_CV CV_32F
//#define LUT_MULTIPLIER
#define LUT_ELEM_TYPE uchar
#define LUT_ELEM_TYPE_CV CV_8U
#define LUT_MULTIPLIER 255 *
#define LUT3D_SIZE 256

/*!
 *\brief   returns a string with information about the image
 *(size, depth...)
 */
inline std::string infosImage(const cv::Mat & i) {
  std::ostringstream concl;
  concl << "size:(" << i.cols << "x" << i.rows << "), ";
  concl << i.channels() << " channels";
  concl << ", depth:" << i.depth();
  concl << ", type:" << i.type();
  concl << ", isContinuous:" << i.isContinuous();
  return concl.str();
}

class ColorFaceDetector {
public:
  static const unsigned int HIST_BINS = 50;

  //! if x < this value; we approximate exp(x) = 0
  static const float EXP_MIN_NON_ZERO_VALUE = -5;

  typedef std::vector<float> ColorList;

  typedef cv::Mat1f ColorHistogram;

  typedef cv::Mat_<LUT_ELEM_TYPE> LookupTable2D;
  typedef cv::Mat LookupTable3D;

  typedef cv::Mat_<LUT_ELEM_TYPE> Likelihood;

  //////////////////////////////////////////////////////////////////////////////

  ColorFaceDetector() {
    was_gaussian_fitted = false;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
 * Convert a BGR image into a compact color space defined by:
 *  r =  R / (R + G + B)
 *  g =  G / (R + G + B)
 * \param in
 *    the BGR image
 * \param r, g
 *    the output vectors.
 *    They can already be filled with some data
 *    (use \a clear_vecs = false not to empty them then)
 * \param clear_vecs
 *    if true, r and g will be emptied before converting \a in.
 *    Passing false can be used for instance when
 *    batch processing a set of images
 *    and keeping all data in a pair of vectors (r,g).
 */
  void convert_color_space(const cv::Mat3b & in,
                           bool clear_vecs = true) {
    if (clear_vecs) {
      r.clear();
      g.clear();
    }
    unsigned int in_size = in.cols * in.rows;
    unsigned int cols = in.cols, rows = in.rows;
    r.reserve(r.size() + in_size);
    g.reserve(g.size() + in_size);
    for (unsigned int row = 0; row < rows; ++row) {
      // get the address of row
      const cv::Vec3b* in_data = in.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < cols; ++col) {
        if (!(*in_data)[0] && !(*in_data)[1] && !(*in_data)[2]) {
          r.push_back( 0 ); // BGR -> R=2
          g.push_back( 0 );
          ++in_data;
          continue;
        }
        float sum_inv = 1.f /
                        ((*in_data)[0] + (*in_data)[1] + (*in_data)[2]);
        r.push_back( sum_inv * (*in_data)[2]); // BGR -> R=2
        g.push_back( sum_inv * (*in_data)[1]);
        ++in_data;
      } // end loop col
    } // end loop row
  } // end convert_color_space();

  //////////////////////////////////////////////////////////////////////////////

  /*!
 * Make the histogram of two vectors of colors
 * (red and green compact components)
 * \param r, g
 *    the compact color representation of a RGB image,
 *    \see convert_color_space();
 * \param hist
 *    The 2D histogram of (r, g) values
 *    r values are in rows,
 *    g values are in columns
 */
  void train_make_histogram() {
    hist.create(HIST_BINS, HIST_BINS);
    hist.setTo(0);
    if (r.size() != g.size()) {
      printf("train_make_histogram(): r(%i) and g(%i) not same sizes!\n",
             r.size(), g.size());
      return;
    }
    int data_size = r.size();
    for (int i = 0; i < data_size; ++i) {
      // image(row, i)
      ++hist(r[i] * HIST_BINS, g[i] * HIST_BINS);
    } // end loop i
    // normalize
    cv::normalize(hist, hist, 1, 0, cv::NORM_INF); // max value = 1
    //cv::normalize(hist, hist, 1, 0, cv::NORM_L1); // sumn all values = 1
    //printf("norm:%g\n", cv::norm(hist, cv::NORM_L1));
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
 * Fit a 2D gaussian model to 2 1-D vectors
 * \param r, g
 * \param r_mean, g_mean, cov_rr, cov_rg, cov_gg
 *    Mean values and covariances of the Gaussian
 */
  void train_fit_gaussian() {
    unsigned int data_size = r.size();
    if (g.size() != data_size) {
      printf("train_make_histogram(): r(%i) and g(%i) not same sizes!\n",
             r.size(), g.size());
      return;
    }
    cov_matrix2((float*) r.data(), (float*) g.data(), data_size,
                r_mean, g_mean,
                cov_rr, cov_rg, cov_gg);
    was_gaussian_fitted = true;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
 * create a lookup table
 * \param lut
 *    r values are in rows,
 *    g values are in columns
 * \param r_mean
 * \param g_mean
 * \param cov_rr
 * \param cov_rg
 * \param cov_gg
 * \param lut2D_size
 */
  void train_create_lut2D(const unsigned int lut2D_size = 256) {
    lut2D.create(lut2D_size, lut2D_size);
    IplImage lut2D_ipl = lut2D;

    // cache a lot of computation (everything not depending on x, y)
    // for the computing of the Gaussian PDF
    float lut2D_size_inv = 1.f / lut2D_size;
    float sx = sqrt(cov_rr), sy = sqrt(cov_gg);
    float sx2inv = 1.f / cov_rr, sy2inv = 1.f / cov_gg;
    float rho = cov_rg / (sx * sy);
    float one_m_rho2 = 1.f - rho * rho;
    float exp_mul_fac = -1.f / (2 * one_m_rho2);
    float rho_by_sx_sy = -2.f * rho / (sx * sy);

    for (unsigned int r_i = 0; r_i < lut2D_size; ++r_i) {
      float r_diff = lut2D_size_inv * r_i - r_mean;
      for (unsigned int g_i = 0; g_i < lut2D_size; ++g_i) {
        // float g = ((float) r_i) / lut2D_size;
        // image(row, col)
        //      lut(r_i, g_i) = gaussian_pdf(r, ((float) g_i) / lut2D_size,
        //                                   r_mean, g_mean, cov_rr, cov_rg, cov_gg, true);
        float g_diff = lut2D_size_inv * g_i - g_mean;
        // gaussian PDF, using all cached computations
        float val = exp_mul_fac *
                    (r_diff * r_diff * sx2inv
                     + g_diff * g_diff * sy2inv
                     + r_diff * g_diff * rho_by_sx_sy);
        //lut2D(r_i, g_i) =
        CV_IMAGE_ELEM(&lut2D_ipl, uchar, r_i, g_i) =
            (val < EXP_MIN_NON_ZERO_VALUE ? 0 : LUT_MULTIPLIER exp(val));
      } // end loop r
    } // end loop r
  } // end train_create_lut();

  //////////////////////////////////////////////////////////////////////////////

  void train_create_lut3D() {
    int dim[] = {LUT3D_SIZE, LUT3D_SIZE, LUT3D_SIZE};
    lut3D = cv::Mat(3, dim, LUT_ELEM_TYPE_CV);
    //printf("lut:%s\n", infosImage(lut).c_str());

#if 1 // compute values with formula - significantly faster on slow PCs

    // cache a lot of computation (everything not depending on x, y)
    // for the computing of the Gaussian PDF
    float sx = sqrt(cov_rr), sy = sqrt(cov_gg);
    float sx2inv = 1.f / cov_rr, sy2inv = 1.f / cov_gg;
    float rho = cov_rg / (sx * sy);
    float one_m_rho2 = 1.f - rho * rho;
    float exp_mul_fac = -1.f / (2 * one_m_rho2);
    float rho_by_sx_sy = -2.f * rho / (sx * sy);

    LUT_ELEM_TYPE* lut_ptr = (LUT_ELEM_TYPE*) lut3D.ptr(0);
    for (unsigned int r_i = 0; r_i < LUT3D_SIZE; ++r_i) {
      if (r_i % 50 == 0)
        printf("train_create_lut3D(%i, to:%i)\n", r_i, LUT3D_SIZE);
      for (unsigned int g_i = 0; g_i < LUT3D_SIZE; ++g_i) {
        for (unsigned int b_i = 0; b_i < LUT3D_SIZE; ++b_i) {
          float sum_inv = 1.f / (r_i + g_i + b_i);
          float r_diff = sum_inv * r_i - r_mean;
          float g_diff = sum_inv * g_i - g_mean;
          // gaussian PDF, using all cached computations
          float val = exp_mul_fac *
                      (r_diff * r_diff * sx2inv
                       + g_diff * g_diff * sy2inv
                       + r_diff * g_diff * rho_by_sx_sy);
          //lut.at<LUT_ELEM_TYPE>(r_i, g_i, b_i)
          *lut_ptr++
              = (val < EXP_MIN_NON_ZERO_VALUE ? 0 : LUT_MULTIPLIER exp(val));
        } // end loop b
      } // end loop g
    } // end loop r

#else // use 2D LUT

    IplImage lut2D_ipl = lut2D;
    unsigned int lut2D_size = lut2D.cols;
    LUT_ELEM_TYPE* lut_ptr = (LUT_ELEM_TYPE*) lut3D.ptr(0);
    for (unsigned int r_i = 0; r_i < LUT3D_SIZE; ++r_i) {
      if (r_i % 50 == 0)
        printf("train_create_lut3D(%i, to:%i)\n", r_i, LUT3D_SIZE);
      for (unsigned int g_i = 0; g_i < LUT3D_SIZE; ++g_i) {
        for (unsigned int b_i = 0; b_i < LUT3D_SIZE; ++b_i) {
          if (!r_i && !g_i && !b_i) {
            ++lut_ptr;
            continue;
          }
          float sum_inv = 1.f * lut2D_size / (r_i + g_i + b_i);
          //  if (isinf(sum_inv))
          //    continue;
          //lut.at<LUT_ELEM_TYPE>(r_i, g_i, b_i)
          *lut_ptr++
              = CV_IMAGE_ELEM(&lut2D_ipl, uchar,
                              (int) (r_i * sum_inv), (int) (g_i * sum_inv));
        } // end loop b
      } // end loop g
    } // end loop r

#endif

    // take care of the (0, 0, 0) case
    lut3D.at<LUT_ELEM_TYPE>(0, 0, 0) = 0;
  } // end train_create_lut();


  //////////////////////////////////////////////////////////////////////////////

  void train(const std::vector<std::string> & imgs_filenames,
             bool train2D = true, bool train3D = true,
             const unsigned int lut2D_size = 256,
             bool use_haar_detector = true)
  {
    unsigned int nfiles = imgs_filenames.size();
    printf("train(%i filenames, 2D:%i, 3D:%i, use_haar_detector:%i)\n",
           nfiles, train2D, train3D, use_haar_detector);
    bool need_clear_rg = true;

    // face detection stuff
    cv::CascadeClassifier classifier;
    if (use_haar_detector)
      classifier = image_utils::create_face_classifier();
    cv::Mat3b small_img;
    std::vector<cv::Rect> res;

    // read all files
    int nfiles_used_for_training = 0;
    for (unsigned int img_idx = 0; img_idx < nfiles; ++img_idx) {
      if (img_idx % 50 == 0)
        printf("train(): read %i files out of %i\n", img_idx, nfiles);

      cv::Mat img = cv::imread(imgs_filenames[img_idx], CV_LOAD_IMAGE_COLOR);
      if (img.empty()) { // assert file well read
        printf("train():Could not read image '%s'\n", imgs_filenames[img_idx].c_str());
        continue;
      }
      if (!use_haar_detector) { // no face detection: compute r, g
        convert_color_space(img, need_clear_rg); // clear only once r and g
        need_clear_rg = false;
        ++nfiles_used_for_training;
        continue;
      }
      // face detec
      image_utils::detect_with_opencv(img, classifier, small_img, res);
      for (unsigned int idx = 0; idx < res.size(); ++idx) {
        // shrink rec
        res[idx] = geometry_utils::shrink_rec(res[idx], .5);
        convert_color_space(img(res[idx]), need_clear_rg);
        need_clear_rg = false;
        ++nfiles_used_for_training;
        //cv::imshow("training_img", img(res[idx])); cv::waitKey(5);
      } // end loop idx
    } // end loop img_idx
    printf("train(): read all %i files, used %i\n", nfiles, nfiles_used_for_training);

    train_make_histogram();
    train_fit_gaussian();
    if (train2D)
      train_create_lut2D(lut2D_size);
    if (train3D)
      train_create_lut3D();
  } // end train()

  //////////////////////////////////////////////////////////////////////////////

  void train(const cv::Mat3b & training_img,
             bool train2D = true, bool train3D = true,
             const unsigned int lut2D_size = 256) {
    convert_color_space(training_img, true);
    train_make_histogram();
    train_fit_gaussian();
    if (train2D)
      train_create_lut2D(lut2D_size);
    if (train3D)
      train_create_lut3D();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool save_training(const std::string filename = "/tmp/ColorFaceDetector.yaml",
                     bool all_data = true) {
    if (!was_gaussian_fitted) {
      printf("save_training(): you need to fit the Gaussian first!");
      return false;
    }
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
      printf("save_training(): could not open '%s'!\n", filename.c_str());
      return false;
    }
    fs << "r_mean" << r_mean;
    fs << "g_mean" << g_mean;
    fs << "cov_rr" << cov_rr;
    fs << "cov_rg" << cov_rg;
    fs << "cov_gg" << cov_gg;
    if (all_data & !hist.empty())
      fs << "hist" << hist;
    printf("save_training(): succesfully generated '%s'\n", filename.c_str());
    return true;
  } // end save_training();


  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Load the values obtained after training on a big database of images
   * (Google Faces images, prefiltered with Viola Haar face detector).
   * \return true if success
   */
  bool load_default_training() {
    r_mean =  4.3287940394554553e-01;
    g_mean =  3.0793284445820024e-01;
    cov_rr =  5.2001926521577594e-03;
    cov_rg =  -1.7388024477420080e-03;
    cov_gg =  1.1602283170410938e-03;
    was_gaussian_fitted = true;
    // now train
    train_create_lut2D();
    train_create_lut3D();
    return true;
  } // end load_default_training()


  //////////////////////////////////////////////////////////////////////////////

  bool load_training(const std::string filename = "/tmp/ColorFaceDetector.yaml") {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      printf("load_training(): could not open '%s'!\n", filename.c_str());
      return false;
    }
    fs["r_mean"] >> r_mean;
    fs["g_mean"] >> g_mean;
    fs["cov_rr"] >> cov_rr;
    fs["cov_rg"] >> cov_rg;
    fs["cov_gg"] >> cov_gg;
    fs["hist"] >> hist;
    was_gaussian_fitted = true;
    // now train
    train_create_lut2D();
    train_create_lut3D();
    return true;
  } // end load_training();

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Need to have called convert_color_space() on the test image before,
   * so as r and g contain compact representations of the test image colors.
   * \param rows
   *    nb of rows of the test image
   * \param likelihood
   *    the final likelihood of each pixel
   */
  void test_apply_gaussian2D(unsigned int rows,
                             Likelihood & likelihood) {
    if (r.size() != g.size()) {
      printf("train_make_histogram(): r(%i) and g(%i) not same sizes!\n",
             r.size(), g.size());
      return;
    }
    if (lut2D.cols != lut2D.rows) {
      printf("train_make_histogram(): lut (%s) not square!\n",
             infosImage(lut2D).c_str());
      return;
    }
    unsigned int cols = r.size() / rows;
    float mult_factor = (float) lut2D.cols;

    IplImage lut_ipl = (IplImage) lut2D;
    likelihood.create(rows, cols);

    int idx = 0;
    for (unsigned int row = 0; row < rows; ++row) {
      LUT_ELEM_TYPE* lk_data = likelihood.ptr<LUT_ELEM_TYPE>(row);
      for (unsigned int col = 0; col < cols; ++col) {
        //      lk_data[i] = gaussian_pdf(r_data[i], g_data[i],
        //                                  r_mean, g_mean, cov_rr, cov_rg, cov_gg,
        //                                  true);
        lk_data[col] = CV_IMAGE_ELEM
                       (&lut_ipl, LUT_ELEM_TYPE,
                        (int) (r[idx] * mult_factor),
                        (int) (g[idx] * mult_factor));
        ++idx;
      } // end loop col
    } // end loop row
  } //end test_apply_gaussian();

  //////////////////////////////////////////////////////////////////////////////

  void test_apply_gaussian_fast2D(const cv::Mat3b & img,
                                  Likelihood & likelihood) {
    if (lut2D.cols != lut2D.rows) {
      printf("train_make_histogram(): lut (%s) not square!\n",
             infosImage(lut2D).c_str());
      return;
    }
    IplImage lut_ipl = (IplImage) lut2D;
    unsigned int rows = img.rows, cols = img.cols;
    float mult_factor = (float) lut2D.cols;
    likelihood.create(rows, cols);
    for (unsigned int row = 0; row < rows; ++row) {
      const cv::Vec3b* in_data = img.ptr<cv::Vec3b>(row);
      LUT_ELEM_TYPE* lk_data = likelihood.ptr<LUT_ELEM_TYPE>(row);
      for (unsigned int col = 0; col < cols; ++col) {
        float mult_factor2 = mult_factor /
                             ((*in_data)[0] + (*in_data)[1] + (*in_data)[2]);
        //lk_data[col] = lut((*in_data)[0] * mult_factor2, (*in_data)[1] * mult_factor2);
        // row, col _ red, green
        if (isinf(mult_factor2)) // R = G = B = 0
          lk_data[col] = 0;
        else
          lk_data[col] = CV_IMAGE_ELEM // row, col
                         (&lut_ipl, LUT_ELEM_TYPE,
                          (int) ((*in_data)[2] * mult_factor2),  // BGR -> R=2
              (int) ((*in_data)[1] * mult_factor2));
        ++in_data;
      } // end loop col
    } // end loop row
  } //end test_apply_gaussian_fast2D();

  //////////////////////////////////////////////////////////////////////////////

  void test_apply_gaussian_fast3D(const cv::Mat3b & img,
                                  Likelihood & likelihood) {
    // apply lookup table
    //cv::LUT(img,lut,likelihood);
    unsigned int rows = img.rows, cols = img.cols;
    likelihood.create(rows, cols);

    assert(lut3D.isContinuous());
    const LUT_ELEM_TYPE* lut_data = lut3D.ptr(0);
    int LUT3D_SIZE2 = LUT3D_SIZE * LUT3D_SIZE;

    for (unsigned int row = 0; row < rows; ++row) {
      const cv::Vec3b* in_data = img.ptr<cv::Vec3b>(row);
      LUT_ELEM_TYPE* lk_data = likelihood.ptr<LUT_ELEM_TYPE>(row);
      for (unsigned int col = 0; col < cols; ++col) {
        lk_data[col] =
            //lut[(*in_data)[2]] [(*in_data)[1]] [(*in_data)[0]];
            //lut.at<LUT_ELEM_TYPE>((*in_data)[2], (*in_data)[1], (*in_data)[0]); //RGB
            lut_data[LUT3D_SIZE2 * (*in_data)[2]  + LUT3D_SIZE * (*in_data)[1] + (*in_data)[0]];
        ++in_data;
      } // end loop col
    } // end loop row
  } //end test_apply_gaussian_fast3D();

  //////////////////////////////////////////////////////////////////////////////

  double test_get_likelihood(Likelihood & likelihood) {
    return (cv::mean(likelihood))[0] / (LUT_MULTIPLIER 1.f);
  }


  //////////////////////////////////////////////////////////////////////////////

  static void likelihood2greyscale(const cv::Mat_<float> & likelihood,
                                   cv::Mat1b & likelihood_greyscale) {
    likelihood.convertTo(likelihood_greyscale, CV_8U, 255, 0);
  }

  static void likelihood2greyscale(const cv::Mat_<uchar> & likelihood,
                                   cv::Mat1b & likelihood_greyscale) {
    likelihood_greyscale = likelihood;
  }

  static cv::Mat1b likelihood2greyscale(const Likelihood & likelihood) {
    cv::Mat1b likelihood_greyscale;
    likelihood2greyscale(likelihood, likelihood_greyscale);
    return likelihood_greyscale;
  }

  //////////////////////////////////////////////////////////////////////////////

  void show_histogram_as_image(bool imshow = true) {
    show_histogram_as_image(hist, imshow);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void show_histogram_as_image(const ColorHistogram & hist, bool imshow = true) {
    if (!hist.isContinuous()) {
      printf("show_2d_histogram(): hist (%s) is not continuous!\n",
             infosImage(hist).c_str());
      return;
    }
    if (hist.cols != (int) HIST_BINS || hist.rows != (int) HIST_BINS) {
      printf("show_2d_histogram(): hist (%s) not with dims %ix%i!\n",
             infosImage(hist).c_str(), HIST_BINS, HIST_BINS);
      return;
    }
    float data[HIST_BINS][HIST_BINS];
    memcpy(data, hist.data, HIST_BINS * HIST_BINS * sizeof(float));
    cv::Mat hist_illus;
    int width1 = 500 / HIST_BINS;
    array_to_color(data, HIST_BINS, HIST_BINS, hist_illus, width1, width1, true, false,
                   colormaps::ratio2red_green);
    if (!imshow)
      return;
    cv::imshow("2d_histogram", hist_illus);
    cv::waitKey(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void show_histogram_gnuplot(bool imshow = true) {
    if (was_gaussian_fitted)
      show_histogram_gnuplot(plot, hist, imshow, r_mean, g_mean, cov_rr, cov_rg, cov_gg);
    else
      show_histogram_gnuplot(plot, hist, imshow);
  }

  //////////////////////////////////////////////////////////////////////////////

  // cf http://mathworld.wolfram.com/GaussianFunction.html
  /*static */void show_histogram_gnuplot
  (Gnuplot & plot,
   const ColorHistogram & hist, bool imshow = true,
   const double & r_mean = -1, const double & g_mean = -1,
   const double & cov_rr = -1, const double & cov_rg = -1, const double & cov_gg = -1)
  {
    plot.reset_all();
    plot.set_xlabel("r");
    plot.set_ylabel("g");
    plot.set_grid();
    plot.cmd("set xyplane at -8e-6"); // z axis starting at 0
    plot.set_xrange(0, 1);
    plot.set_yrange(0, 1);
    plot.set_zrange(0, 1);

    // plot histogram
    if (hist.cols != (int) HIST_BINS || hist.rows != (int) HIST_BINS) {
      printf("show_histogram_gnuplot(): hist (%s) not with dims %ix%i, not plotting it!\n",
             infosImage(hist).c_str(), HIST_BINS, HIST_BINS);
    }
    else if (!hist.isContinuous()) {
      printf("show_histogram_gnuplot(): hist (%s) is not continuous!\n",
             infosImage(hist).c_str());
    }
    else {
      // make a nice (r, g, z) 3D cloud
      std::vector<float> g_vec, r_vec, z_vec;
      g_vec.reserve(HIST_BINS * HIST_BINS);
      r_vec.reserve(HIST_BINS * HIST_BINS);
      z_vec.reserve(HIST_BINS * HIST_BINS);
      float hist_bin_inv = 1.f / HIST_BINS;
      // r in rows, g in cols
      for (unsigned int row = 0; row < HIST_BINS; ++row) {
        const float* hist_data = hist.ptr<float>(row);
        float r_value = (row + .5f) * hist_bin_inv; // center data
        for (unsigned int col = 0; col < HIST_BINS; ++col) {
          r_vec.push_back(r_value);
          g_vec.push_back((col + .5f) * hist_bin_inv);
          z_vec.push_back(hist_data[col]);
        } // end loop x
      } // end loop y

      std::ostringstream style;
      style << "impulses linewidth " << (int) (200 / HIST_BINS);
      plot.set_style(style.str()); // bars for values
      plot.set_zlabel("hist");
      plot.plot_xyz(r_vec, g_vec, z_vec);
    }

    // plot gaussian func
    if (r_mean == -1 && g_mean == -1 && cov_rr == -1 && cov_rg == -1 && cov_gg == -1) {
      printf("Not plotting gaussian\n");
    }
    else {
      plot.set_style("lines");
      //plot.cmd("set pm3d"); // 3d filled surface
      //plot.unset_grid();
      //plot.cmd("set style fill  transparent solid 0.30");
      plot.cmd("set isosamples 25,25");
      plot.plot_equation3d(gaussian_pdf_as_string(r_mean, g_mean, cov_rr, cov_rg, cov_gg, true));
    }

    if (!imshow)
      return;
    //plot.cmd("pause -1");
    system_utils::wait_for_key();
  } // end show_histogram_gnuplot();

  //////////////////////////////////////////////////////////////////////////////

  std::string gaussian_params() {
    std::ostringstream ans;
    ans << "mean:(" << r_mean << ", " << g_mean << ")"
        << ", cov:(" << cov_rr<< ", " << cov_rg
        << ", " << cov_rg<< ", " << cov_gg << ")";
    return ans.str();
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string gaussian_pdf_equa() {
    return gaussian_pdf_as_string(r_mean, g_mean, cov_rr, cov_rg, cov_gg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void show_training(bool imshow = true) {
    if (!was_gaussian_fitted) {
      printf("show_training(): you need to fit the Gaussian first!");
    }
    printf("ColorFaceDetector:gaussian:%s -> PDF(x, y):%s\n",
           gaussian_params().c_str(),
           gaussian_pdf_equa().c_str());
    show_histogram_gnuplot(false);
    cv::imshow("lut_greyscale",  ColorFaceDetector::likelihood2greyscale(lut2D));
    if (imshow)
      cv::waitKey(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  //private:
  LookupTable2D lut2D;
  LookupTable3D lut3D;
  ColorList r, g;
  ColorHistogram hist;
  bool was_gaussian_fitted;
  double r_mean, g_mean, cov_rr, cov_rg, cov_gg;
  Gnuplot plot;
}; // end class ColorFaceDetector


#endif // COLOR_FACE_DETECTOR_H
