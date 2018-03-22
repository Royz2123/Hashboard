//
// Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
// Published under the MIT license.
//
#pragma once

// Raspberry Camera
#include <raspicam/raspicam_cv.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::Point2f;
using cv::Point2i;
using cv::Point3f;
using cv::Vec3f;
using cv::Vec4f;
using cv::Vec4i;
using cv::Vec4b;
using cv::Size;
using cv::Size2i;
using cv::Range;
using cv::Rect2i;
using cv::Affine3d;
using cv::Affine3f;
using cv::VideoCapture;




bool squareToPosition(vector<Point2f> cameraSquare, Vec3f dronePos);
void findHand(
  Mat frame,
  std::pair<vector<Point2f>, vector<Point2f>> leftHand,
  std::pair<vector<Point2f>, vector<Point2f>> rightHand,
);


int main() {

}



bool squareToPosition(vector<Point2f> cameraSquare, Vec3f dronePos) {
  Mat rvec;
  Mat tvec;
	bool found;

  found = cv::solvePnP(
    this->worldSquare,
    cameraSquare,
    this->cameraMatrix,
    Mat{},
    rvec,
    tvec
  );

	if (!found) {
		return false;
	}

  printTimeSinceLastCall("SolvePnP");

  rvec.convertTo(rvec, CV_32F);
  tvec.convertTo(tvec, CV_32F);

  Affine3f cameraTransform = Affine3f{rvec, tvec};
  // The square doesn't move, we are moving.
  cameraTransform = cameraTransform.inv();

  // find drone position
  dronePos = (cameraTransform * this->invDroneCameraTransform).translation();

  // smooth vector out
  this->smoothVector(dronePos);

	return true;
}
