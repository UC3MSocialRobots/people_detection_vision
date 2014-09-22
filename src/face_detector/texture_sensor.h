/*!
  \file        texture_sensor.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/12
  
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

The texture sensor for face detection, as cited by
Rehg, James M., Kevin P. Murphy, and Paul W. Fieguth.
"Vision-based speaker detection using bayesian networks."
Computer Vision and Pattern Recognition, 1999.
IEEE Computer Society Conference on.. Vol. 2. IEEE, 1999.

 */

#ifndef TEXTURE_SENSOR_H
#define TEXTURE_SENSOR_H

#include <opencv2/highgui/highgui.hpp>
#include <geom/rect_utils.h>
#include "stats/stats_utils.h"

class TextureSensor {
public:
  TextureSensor() {}

  //////////////////////////////////////////////////////////////////////////////

  double correlation_ratio(const cv::Mat1b & img) {
    int cols = img.cols, rows = img.rows;
    int col_incr = cols * COL_INCR_RATIO;
    int last_col = (cols-1) - col_incr;
    // init to correct number of values
    num.clear();
    num.resize(cols * rows);
    denom.clear();
    denom.resize(cols * rows);
    IplImage img_ipl = img;

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col <= last_col; ++col) {
        uchar pixel_val = CV_IMAGE_ELEM(&img_ipl, uchar, row, col);
        num.push_back(pixel_val * CV_IMAGE_ELEM(&img_ipl, uchar, row, col + col_incr));
        denom.push_back(pixel_val * pixel_val);
      } // end loop col
    } // end loop row

    return 1.f * mean(num.data(), num.size()) / mean(denom.data(), denom.size());
  }

  //////////////////////////////////////////////////////////////////////////////

  void mosaic_img(const cv::Mat1b & img_bw,
                  cv::Mat1f & img_corr,
                  int w1 = 0, int h1 = 0) {
    int cols = img_bw.cols, rows = img_bw.rows;
    if (w1 <= 0 || h1 <= 0) {
      printf("mosaic_img:Taking default values for w1, h1\n");
      w1 = cols / 20;
      h1 = rows / 20;
    }
    img_corr.create(img_bw.size());
    for (int row = 0; row < rows; row += h1) {
      for (int col = 0; col < cols; col += w1) {
        cv::Rect rect(col, row, w1, h1);
        rect = geometry_utils::rectangle_intersection_img(img_bw, rect);
        double corr = correlation_ratio(img_bw(rect));
        img_corr(rect) = corr;
      } // end loop col
    } // end loop row
  }

  static const double COL_INCR_RATIO =  1.f / 12;
  std::vector<int> num, denom;
}; // end class TextureSensor

#endif // TEXTURE_SENSOR_H
