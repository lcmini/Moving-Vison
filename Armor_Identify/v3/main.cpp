#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include<stdio.h>
#include "armordetection.h"

using namespace cv;
using namespace std;

ArmorDetection* armor = new ArmorDetection();
Point2f center;

int main()
{
	//fps变量
	double t = (double)getTickCount();
	double fps;
	
	Mat frame;

	VideoCapture capture("C:\\Users\\Lenovo\\Pictures\\car1 79fps kk.mp4");
	if (!capture.isOpened())
	{
		printf("无法打开相机...\n");
		return -1;
	}

	namedWindow("frame", CV_WINDOW_NORMAL);
	namedWindow("mask", CV_WINDOW_NORMAL);
	namedWindow("Control", CV_WINDOW_NORMAL);
	//namedWindow("dst", CV_WINDOW_NORMAL);
	while (capture.read(frame))//读取当前帧
	{
		imshow("haha", frame);
		t = (double)getTickCount();
		armor->setInputImage(frame);
		armor->Pretreatment();
		center = armor->GetArmorCenter();
		t = ((double)getTickCount() - t) / (double)getTickFrequency();
		fps = 1.0 / t;
		cout << "[INFO] x = " << center.x - frame.cols / 2 << "    y = " << center.y - frame.rows / 2 << endl;
 
		cout << "fps is:  " << fps << endl;
		
		 
		 

		char c = waitKey(10);
		if (c == 27) //"Esc"
		{
			break;
		}
	}
	capture.release();//释放视频内存
	waitKey(0);
	return 0;
}