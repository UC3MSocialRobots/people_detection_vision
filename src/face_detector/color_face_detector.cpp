/*!
  \file        color_face_detector.cpp
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

#include "color_face_detector.h"
#include <vision_utils/img_path.h>
#include "src/time/timer.h"
#include "src/combinatorics/combinatorics_utils.h"

void train(const std::string & training_filename) {
  ColorFaceDetector detec;
  Timer timer;
  unsigned int ntimes = 10;
  cv::Mat3b training_img = cv::imread(training_filename, CV_LOAD_IMAGE_COLOR);

#if 1 // verbose training
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.convert_color_space(training_img);
  timer.printTime_factor("convert_color_space()", ntimes);
  //std::cout << "r:" << StringUtils::accessible_to_string(r) << std::endl << std::endl;
  //std::cout << "g:" << StringUtils::accessible_to_string(g) << std::endl << std::endl;

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.train_make_histogram();
  timer.printTime_factor("train_make_histogram()", ntimes);
  //std::cout << "hist:" << hist << std::endl << std::endl;

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.train_fit_gaussian();
  timer.printTime_factor("train_fit_gaussian()", ntimes);
  printf("gaussian:%s -> PDF(x, y):%s\n",
         detec.gaussian_params().c_str(),
         detec.gaussian_pdf_equa().c_str());

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.train_create_lut2D(256);
  timer.printTime_factor("train_create_lut2D()", ntimes);
  // std::cout << "lut:" << lut << std::endl;

  timer.reset();
  //for (unsigned int time = 0; time < ntimes; ++time)
  detec.train_create_lut3D();
  //timer.printTime_factor("train_create_lut3D()", ntimes);
  timer.printTime("train_create_lut3D()");

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.show_histogram_as_image(false);
  timer.printTime_factor("show_histogram_as_image()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    detec.show_histogram_gnuplot(false);
  timer.printTime_factor("show_histogram_gnuplot()", ntimes);

#else

  for (unsigned int time = 0; time < ntimes; ++time)
    detec.train(training_img);
  timer.printTime_factor("train()", ntimes);

#endif

  detec.save_training();
  detec.show_training();
}

////////////////////////////////////////////////////////////////////////////////

void train(const std::vector<std::string> & imgs_filenames) {
  ColorFaceDetector detec;
  Timer timer;
  detec.train(imgs_filenames);
  timer.printTime("train()");
  detec.save_training();
  detec.show_training();
}


////////////////////////////////////////////////////////////////////////////////

void test_on_image(const std::string & test_filename) {
  ColorFaceDetector detec;
  if (!detec.load_training()) {
    printf("Loading training failed. Loading default training.\n");
    detec.load_default_training();
  }

  unsigned int ntimes = 10;
  cv::Mat3b test_img = cv::imread(test_filename, CV_LOAD_IMAGE_COLOR);

  Timer timer;
  ColorFaceDetector::Likelihood likelihood;
  for (unsigned int time = 0; time < ntimes; ++time) {
    detec.convert_color_space(test_img);
    detec.test_apply_gaussian2D(test_img.rows, likelihood);
  }
  timer.printTime_factor("test_apply_gaussian2D()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time) {
    detec.test_apply_gaussian_fast2D(test_img, likelihood);
  }
  timer.printTime_factor("test_apply_gaussian_fast2D()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time) {
    detec.test_apply_gaussian_fast3D(test_img, likelihood);
  }
  timer.printTime_factor("test_apply_gaussian_fast3D()", ntimes);

  //return;
  cv::imshow("likelihood",  ColorFaceDetector::likelihood2greyscale(likelihood));
  detec.show_training();
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

void test_on_cam(int device = 0) {
  ColorFaceDetector detec;
  Timer timer;
  if (!detec.load_training()) {
    printf("Loading training failed. Loading default training.\n");
    detec.load_default_training();
  }

  timer.printTime("load_training();");
  detec.show_training(false);

  cv::VideoCapture capture(device);
  ColorFaceDetector::Likelihood likelihood(1, 1);
  cv::Mat1b likelihood_greyscale;
  bool mode3D = true;

  while(capture.isOpened()) {
    cv::Mat3b test_img;
    capture >> test_img;
    timer.reset();
    if (mode3D) {
      detec.test_apply_gaussian_fast3D(test_img, likelihood);
      timer.printTime("test_apply_gaussian_fast3D();");
    } // end if (mode3D)
    else {
      detec.test_apply_gaussian_fast2D(test_img, likelihood);
      timer.printTime("test_apply_gaussian_fast2D();");
    } // end if (mode2D)
    ColorFaceDetector::likelihood2greyscale(likelihood, likelihood_greyscale);
    cv::imshow("likelihood", likelihood_greyscale);
    cv::imshow("test_img", test_img);
    char c = (char) cv::waitKey(5);
    if (c == 'm')
      mode3D = !mode3D;
  } // end while(capture.isOpened())
} // end test_on_cam();

////////////////////////////////////////////////////////////////////////////////

// from http://cs229.stanford.edu/section/gaussians.pdf
void foo_test_train_fit_gaussian() {
  unsigned int nvalues = 100;

  ColorFaceDetector detec;
  detec.r.resize(nvalues);
  detec.g.resize(nvalues);
  double xm = .2, ym = .7;
  for (unsigned int i= 0; i < nvalues; ++i) {
    // image(row, col)
    detec.r[i] = std::min(1., std::max(0., xm + combinatorics_utils::rand_gaussian() / 10));
    detec.g[i] = std::min(1., std::max(0., ym + combinatorics_utils::rand_gaussian() / 10));
  } // end loop i
  std::cout << "r:" << StringUtils::accessible_to_string(detec.r) << std::endl << std::endl;
  std::cout << "g:" << StringUtils::accessible_to_string(detec.g) << std::endl << std::endl;

  detec.train_make_histogram();
  detec.show_histogram_gnuplot(true);
  std::cout << "hist:" << detec.hist<< std::endl;

  detec.train_fit_gaussian();
  printf("gaussian:%s -> PDF(x, y):%s\n",
         detec.gaussian_params().c_str(),
         detec.gaussian_pdf_equa().c_str());
  detec.show_histogram_gnuplot(true);
  // interesting for gnuplot
  // https://groups.google.com/forum/#!topic/comp.graphics.apps.gnuplot/uLSmHWK_xtE
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  srand(time(NULL));

  int idx = 1;
  if (argc < 2) {
    printf("%i: train(IMG_DIR notebook.png)\n", idx++);
    printf("%i: train(IMG_DIR arnaud_cut001.png)\n", idx++);
    printf("%i: train(all_files_in_dir(/home/user/Downloads/0datasets/google_genders/man));\n", idx++);
    printf("%i: train(all_files_in_dir(IMG_DIR faces_color/google));\n", idx++);
    printf("%i: foo_test_train_fit_gaussian()\n", idx++);
    printf("%i: test_on_image(IMG_DIR arnaud001.png)\n", idx++);
    printf("%i: test_on_image(IMG_DIR arnaud002.png)\n", idx++);
    printf("%i: test_on_cam()\n", idx++);
    return -1;
  }
  int choice = 1;
  choice = atoi(argv[1]);
  //std::cin >> choice;

  idx = 1;
  if (choice == idx++)
    train(IMG_DIR "notebook.png");
  else if (choice == idx++)
    train(IMG_DIR "arnaud_cut001.png");
  else if (choice == idx++) {
    std::vector<std::string> files;
    system_utils::all_files_in_dir("/home/user/Downloads/0datasets/google_genders/man", files);
    train(files);
  }
  else if (choice == idx++) {
    std::vector<std::string> files;
    system_utils::all_files_in_dir(IMG_DIR "faces_color/google", files);
    train(files);
  }
  else if (choice == idx++)
    foo_test_train_fit_gaussian();
  else if (choice == idx++)
    test_on_image(IMG_DIR "arnaud001.png");
  else if (choice == idx++)
    test_on_image(IMG_DIR "arnaud002.png");
  else if (choice == idx++)
    test_on_cam(argc >= 3 ? atoi(argv[2]) : 0);
  return 0;
}
