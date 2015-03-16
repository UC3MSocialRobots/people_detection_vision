/*!
  \file        texture_sensor.cpp
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

Some tests for the texture sensor.
 */

#include "texture_sensor.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vision_utils/img_path.h>
#include "compressed_rounded_image_transport/cv_conversion_float_uchar.h"

#include <time/timer.h>

////////////////////////////////////////////////////////////////////////////////

struct GuiData {
  const cv::Mat3b* img;
  const cv::Mat1b* img_bw;
  cv::Mat3b* img_out;
  TextureSensor* sensor;
};

void mouse_cb(int event, int x, int y, int flags, void* userdata) {
  GuiData* data = (GuiData*) userdata;
  data->img->copyTo(*(data->img_out));
  int w = 50, h = 100;
  cv::Rect rect = geometry_utils::rectangle_intersection_img
                  (*(data->img), cv::Rect (x - w / 2, y - h / 2, w, h));
  // draw rect
  cv::rectangle(*(data->img_out), rect, CV_RGB(0, 255, 0), 2);
  // send data to texture detector
  double corr = data->sensor->correlation_ratio((*data->img_bw)(rect));
  std::ostringstream text; text << corr;
  cv::putText(*(data->img_out), text.str(), cv::Point(x, y), CV_FONT_HERSHEY_PLAIN,
              1, CV_RGB(0, 255, 0));
}

void gui(const cv::Mat3b & img) {
  // create data to be given to cb
  cv::Mat1b img_bw;
  cv::cvtColor(img, img_bw, CV_BGR2GRAY);
  cv::Mat3b img_out;
  img.copyTo(img_out);
  TextureSensor sensor;

  GuiData data;
  data.img = &img;
  data.img_bw = &img_bw;
  data.img_out = &img_out;
  data.sensor = &sensor;

  std::string win_name = "texture_sensor";
  cv::namedWindow(win_name);
  cv::setMouseCallback(win_name, mouse_cb, &data  );

  while (true) {
    cv::imshow(win_name, img_out);
    char c = cv::waitKey(5);
    if ((int) c == 27)
      break;
  } // end while (true)
}

////////////////////////////////////////////////////////////////////////////////

void mosaic(const cv::Mat3b & img) {
  cv::Mat1f img_corr;
  cv::Mat1b img_bw;
  cv::cvtColor(img, img_bw, CV_BGR2GRAY);
  TextureSensor sensor;
  sensor.mosaic_img(img_bw, img_corr);
  cv::imshow("img_bw", img_bw);
  cv::imshow("mosaic", image_utils::depth2viz(img_corr, image_utils::FULL_RGB_STRETCHED));
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

void mosaic_cam(int device = 0) {
  cv::VideoCapture capture(device);
  TextureSensor sensor;
  cv::Mat1f img_corr;
  cv::Mat1b img_bw;
  cv::Mat3b float_out_color;

  std::string win_name = "mosaic";
  cv::namedWindow(win_name);
  int w1 = 20, h1 = 40;
  cv::createTrackbar("h1", win_name, &h1, 200);
  cv::createTrackbar("w1", win_name, &w1, 200);

  while(capture.isOpened()) {
    cv::Mat3b img;
    capture >> img;
    cv::cvtColor(img, img_bw, CV_BGR2GRAY);
    Timer timer;
    sensor.mosaic_img(img_bw, img_corr, w1, h1);
    timer.printTime("mosaic_img()");

    image_utils::depth_image_to_vizualisation_color_image
        (img_corr, float_out_color, image_utils::FULL_RGB_STRETCHED);

    cv::imshow("img_bw", img_bw);
    cv::imshow(win_name, float_out_color);
    char c = (char) cv::waitKey(5);
    if ((int) c == 27)
      break;
  } // end while(capture.isOpened())
} // end mosaic_cam();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1;
  if (argc < 2) {
    printf("%i: gui(IMG_DIR arnaud001.png)\n", idx++);
    printf("%i: mosaic(IMG_DIR arnaud001.png)\n", idx++);
    printf("%i: mosaic_cam()\n", idx++);
    return -1;
  }

  int choice = 0;
  choice = atoi(argv[1]);

  idx = 1;
  if (choice == idx++)
    gui(cv::imread(IMG_DIR "arnaud001.png"));
  else if (choice == idx++)
    mosaic(cv::imread(IMG_DIR "arnaud001.png"));
  else if (choice == idx++)
    mosaic_cam();

  return 0;
}
