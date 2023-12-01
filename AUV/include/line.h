#ifndef LINE_H
#define LINE_H
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int find_line(Mat frame,int start);
extern int C_X,C_Y,k;
#endif // LINE_H
