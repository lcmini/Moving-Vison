#ifndef BALL_H
#define BALL_H
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

char BlueBallSearch(Mat SrcMat, Point3i *TargetErr, int FindNum, int &ResultNum);
char GreenBallSearch(Mat SrcMat, Point3i *TargetErr, int FindNum, int &ResultNum);
#endif // BALL_H
