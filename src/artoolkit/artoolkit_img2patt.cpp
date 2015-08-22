/*!
  \file        artoolkit_img2pat.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/10
  
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

ARToolKit ( http://www.hitl.washington.edu/artoolkit/ )
is a computer tracking library for creation of augmented reality
applications that overlay virtual imagery on the real world.

It works with so-called pattern files, representing the visual appearance
of the markers. These patterns are then searched in the input stream of image.

This program can transform color or BW images into ARToolkit patterns.

 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/border_remover.h"
#include "vision_utils/utils/filename_handling.h"
#include "vision_utils/utils/file_io.h"

bool img2pat(const std::string & imgfilein,
             const std::string & patfileout,
             int cols = 64) {
  cv::Mat3b img = cv::imread(imgfilein, CV_LOAD_IMAGE_COLOR);
  if (img.empty()) {
    printf("Cant read '%s'\n", imgfilein.c_str());
    return false;
  }
  // remove border
  cv::Mat3b img_noborder;
  image_utils::remove_border(img, img_noborder, cv::Vec3b(0, 0, 0));
  cv::imshow("img_noborder", img_noborder); cv::waitKey(100);

  cv::Mat3b img_resized;
  cv::resize(img_noborder, img_resized, cv::Size(cols, cols));

  std::ostringstream out;
  cv::Mat3b img_rotated = img_resized.clone();
  img_resized.copyTo(img_rotated);
  for (unsigned int rotate_idx = 0; rotate_idx < 4; ++rotate_idx) {
    cv::imshow("img_rotated", img_rotated); cv::waitKey(100);
    std::ostringstream r, g, b;
    for (int row = 0; row < cols; ++row) {
      cv::Vec3b* data = img_rotated.ptr<cv::Vec3b>(row);
      for (int col = 0; col < cols; ++col) {
        b << std::setw(4) << std::setfill(' ') << (int) data[col][0];
        g << std::setw(4) << std::setfill(' ') << (int) data[col][1];
        r << std::setw(4) << std::setfill(' ') << (int) data[col][2];
      } // end loop col
      b << std::endl;
      g << std::endl;
      r << std::endl;
    } // end loop row
    // rotate of 90 degrees
    cv::transpose(img_rotated, img_rotated);
    cv::flip(img_rotated, img_rotated, 0);

    // ARToolKit files are encoded in BGR
    // seen thanks to samples obtained on http://flash.tarotaro.org/blog/2009/07/12/mgo2/
    out << b.str() << g.str() << r.str();
    out << std::endl;
  } // end loop rotate_idx

  string_utils::save_file(patfileout, out.str());
  printf("Written file '%s'\n", patfileout.c_str());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Synopsis: %s <imgfilein> [patfileout] [cols]\n", argv[0]);
    return -1;
  }
  std::string imgfilein = argv[1];
  std::string patfileout = (argc >= 3 ? argv[2] :
                            string_utils::change_filename_extension(imgfilein,  ".patt"));
  int cols = (argc >= 4 ? atoi(argv[3]) : 16);
  bool ok = img2pat(imgfilein, patfileout, cols);
  return (ok ? 0 : -1);
}
