/**
 * @file objectDetection2.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to find objects (Face + eyes)in a video stream - Using LBP here
 */
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/core/utility.hpp"

#include "opencv2/highgui/highgui_c.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
#include <vision_utils/img_path.h>
string face_cascade_name = IMG_DIR "haarCascade_openCV/lbpcascade_frontalface.xml";
string eyes_cascade_name = IMG_DIR "haarCascade_openCV/haarcascade_eye_tree_eyeglasses.xml";
//string eyes_cascade_name = IMG_DIR "haarCascade_openCV/haarcascade_mcs_mouth.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";

RNG rng(12345);

/**
 * @function main
 */
int main( void )
{
  CvCapture* capture;
  Mat frame;

  //-- 1. Load the cascade
  if( !face_cascade.load( face_cascade_name )){ printf("--(!)Error loading\n"); return -1; };
  if( !eyes_cascade.load( eyes_cascade_name )){ printf("--(!)Error loading\n"); return -1; };

  //-- 2. Read the video stream
  capture = cvCaptureFromCAM( -1 );
  if( capture )
  {
    for(;;)
    {
      frame = cv::cvarrToMat(cvQueryFrame( capture ));

      //-- 3. Apply the classifier to the frame
      if( !frame.empty())
      { detectAndDisplay( frame ); }
      else
      { printf(" --(!)No captured frame -- Break!"); break; }

      int c = waitKey(10);
      if( (char)c == 'c' ){ break; }

    }
  }
  return 0;
}

/**
 * @function detectAndDisplay
 */
void detectAndDisplay( Mat frame )
{
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0, Size(40, 40));

  for( size_t i = 0; i < faces.size(); i++ )
  {
    Mat faceROI = frame_gray( faces[i] );
    std::vector<Rect> eyes;

    //-- In each face, detect eyes
    eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30));
    if( eyes.size()== 2)
    {
      //-- Draw the face
      Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
      ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 0 ), 2, 8, 0 );

      for( size_t j = 0; j < eyes.size(); j++ )
      { //-- Draw the eyes
        Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
        int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
        circle( frame, eye_center, radius, Scalar( 255, 0, 255 ), 3, 8, 0 );
      }
    }

  }
  //-- Show what you got
  imshow( window_name, frame );
}