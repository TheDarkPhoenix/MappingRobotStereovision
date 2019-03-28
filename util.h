#ifndef UTIL_H
#define UTIL_H

#include <windows.h>
#include <conio.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "types.h"

using namespace cv;

int sendread(HANDLE hSerial, uint8_t l);
void init(HANDLE& hSerial, DCB& dcbSerialParams, COMMTIMEOUTS& timeouts);

bool initStereovision(StereoSGBM& sgbm, VideoCapture& cap1, VideoCapture& cap2, Mat& map11, Mat& map12, Mat& map21, Mat& map22, Mat& Q, int& numberOfDisparities);
void CallBackFunc(int event, int x, int y, int flags, void* disp);

#endif // UTIL_H

