#ifndef REC_H
#define REC_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int rec(Mat frame,int start);
extern int REC_X,REC_Y;
#endif // REC_H
