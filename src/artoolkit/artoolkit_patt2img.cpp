/*!
  \file        artoolkit_patt2img.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/11

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

This program can transform ARToolKit patterns into images for visualizing them.

*/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/border_remover.h"
#include "vision_utils/utils/find_and_replace.h"
#include "vision_utils/utils/string_split.h"
#include "vision_utils/utils/filename_handling.h"
#include "vision_utils/utils/file_io.h"

inline void clean_line(std::string & line) {
  string_utils::remove_beginning_spaces(line);
  string_utils::remove_trailing_spaces(line);
  // remove double spaces
  while (string_utils::find_and_replace(line, "  ", " ")) {}
} // end clean_line();

////////////////////////////////////////////////////////////////////////////////

bool read_line(const std::vector<std::string> & lines,
               const unsigned int row, const unsigned int nrows,
               std::vector<int> & r, std::vector<int> & g, std::vector<int> & b) {
  // ARToolKit files are encoded in BGR
  // seen thanks to samples obtained on http://flash.tarotaro.org/blog/2009/07/12/mgo2/
  std::string bline = lines[row], gline = lines[row+nrows], rline = lines[row+2*nrows];
  clean_line(rline);
  clean_line(gline);
  clean_line(bline);
  string_utils::StringSplit_<int>(rline, " ", &r);
  string_utils::StringSplit_<int>(gline, " ", &g);
  string_utils::StringSplit_<int>(bline, " ", &b);
  if (r.size() != nrows || g.size() != nrows || b.size() != nrows) {
    printf("Nb of values for row %i: r:%i, g:%i b:%i,  and not %i!\n",
           row, r.size(), g.size(), b.size(), nrows);
    return false;
  }
  return true;
} // end read_line()

////////////////////////////////////////////////////////////////////////////////

bool pat2img(const std::string & patfilein,
             const std::string & imgfileout) {
  std::vector<std::string> lines;
  string_utils::retrieve_file_split(patfilein, lines);
  unsigned int nlines = lines.size(), row = 0;
  if (nlines == 0) {
    printf("Cant read '%s', file non existing or empty.\n", patfilein.c_str());
    return false;
  }

  // determine number of cols
  while (row < nlines && lines[row].size() > 0) {
    ++row;
  } // end loop row
  if (row % 3 != 0) {
    printf("The first empty line (#%i) is not a multiple of 3!\n", row);
  }
  unsigned int nrows = row / 3;
  printf("nrows:%i\n", nrows);

  // now read each line
  std::vector<int> r, g, b;
  cv::Mat3b img_out(nrows, nrows);
  img_out.setTo(0);
  for (row = 0; row < nrows; ++row) {
    cv::Vec3b* data = img_out.ptr<cv::Vec3b>(row);
    for (unsigned int col = 0; col < nrows; ++col) {
      if (!read_line(lines, row, nrows, r, g, b))
          continue;
      data[col][0] = b[col];
      data[col][1] = g[col];
      data[col][2] = r[col];
    } // end loop col
  } // end loop row
  cv::Mat3b img_big(256, 256);
  cv::resize(img_out, img_big, img_big.size(), 0, 0, CV_INTER_NN);

  printf("Written img '%s'\n", imgfileout.c_str());
  cv::imshow("img_big", img_big); cv::imshow("img_out", img_out); cv::waitKey(0);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Synopsis: %s <patfilein> [imgfileout]\n", argv[0]);
    return -1;
  }
  std::string patfilein = argv[1];
  std::string imgfileout = (argc >= 3 ? argv[2] :
                            string_utils::change_filename_extension(patfilein,  "_viz.png"));
  bool ok = pat2img(patfilein, imgfileout);
  return (ok ? 0 : -1);
}
