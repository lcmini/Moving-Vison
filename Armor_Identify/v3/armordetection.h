#pragma once
#ifndef ARMORDETECTION_H
#define ARMORDETECTION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include<stdio.h>

using namespace std;
using namespace cv;

 

class  ArmorDetection {
private:
	Mat frame, thresholdImage, channels, mask;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Mat kernel2 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	Point2f currentCenter;
	Point2f lastCenter;
	vector<RotatedRect> minRects;
	char str[100];
	char str2[100];
	float center[2];

	int lost;

	//pre-treatment
	//brightness_threshold = 210;
	//color_threshold = 40;
	//light_color_detect_extend_ratio = 1.1;

	// Filter lights
	//light_min_area = 10;
	//light_max_angle = 45.0;
	//light_min_size = 5.0;
	//light_contour_min_solidity = 0.5;
	//light_max_ratio = 1.0;

	// Filter pairs
	float light_max_angle_diff_ = 7.0; //20
	float light_max_height_diff_ratio_ = 0.2; //0.5
	float light_max_y_diff_ratio_ = 2.0; //100
	float light_min_x_diff_ratio_ = 0.5; //100

	// Filter armor
	float armor_big_armor_ratio = 3.2;
	float armor_small_armor_ratio = 2;

	//armor_max_height_ = 100.0;
	//armor_max_angle_ = 30.0;
	float armor_min_aspect_ratio_ = 1.0;
	float armor_max_aspect_ratio_ = 5.0;



public:
	ArmorDetection();
	explicit ArmorDetection(Mat& input);
	void setInputImage(Mat input);
	void Pretreatment();
	Point2f GetArmorCenter();
	 


private:
	void LostTarget();
	double Distance(Point2f, Point2f);
	double max(double, double);
	double min(double, double);
};

 


#endif 


















 