#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs.hpp>

template<typename T1>
constexpr auto DIS(T1 A, T1  B) { return sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2)); }
struct ARMOR {
	float point[3];
	cv::Point2f pt[4];
	cv::Point2f center;
	long double angle;
	bool num;//num=0:1  num=1:2
	bool camp;//camp=0:friend(blue) camp=1:enemy(red)
};
cv::Mat bkg(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

///////////////////////////////////////////////
float get_depth_scale(rs2::device dev);
bool JUI(cv::RotatedRect A, cv::RotatedRect B);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

std::vector<cv::RotatedRect> process(cv::Mat src_ori);
std::vector<ARMOR> distinguish(std::vector<cv::RotatedRect> lightbar);
void getinfo(cv::Mat src_color, std::vector<ARMOR>& armor);
void visualization(const char* color_win_name, cv::Mat src_color, const char* depth_win_name, cv::Mat src_depth, std::vector<cv::RotatedRect> lightbar, std::vector<ARMOR> armor);
void pixiv_to_point(std::vector<ARMOR>& armor, rs2::depth_frame aligned_depth_frame, rs2_intrinsics depth_intrin);

//////////////////////////////////////////////
int main(int argc, char* argv[]) try
{
	const char* depth_win = "depth_Image";
	cv::namedWindow(depth_win, cv::WINDOW_AUTOSIZE);
	const char* color_win = "color_Image";
	cv::namedWindow(color_win, cv::WINDOW_AUTOSIZE);

	rs2::colorizer color_map;
	rs2::pipeline pipe;
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
	pipe_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	rs2::pipeline_profile profile = pipe.start(pipe_config);

	auto sensor = profile.get_device().query_sensors()[1];
	sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, false);
	sensor.set_option(RS2_OPTION_EXPOSURE, 120);

	float depth_scale = get_depth_scale(profile.get_device());
	rs2_stream align_to = RS2_STREAM_COLOR;
	rs2::align align(align_to);
	float depth_clipping_distance = 1.f;

	while (cv::waitKey(1) < 0 && cv::getWindowProperty(depth_win, cv::WND_PROP_AUTOSIZE) >= 0) {
		rs2::frameset frameset = pipe.wait_for_frames();
		if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams())) {
			profile = pipe.get_active_profile();
			align = rs2::align(align_to);
			depth_scale = get_depth_scale(profile.get_device());
		}
		auto processed = align.process(frameset);
		rs2::depth_frame aligned_depth_frame_ori = processed.get_depth_frame();
		rs2::frame aligned_color_frame = processed.get_color_frame();
		rs2::frame aligned_depth_frame = processed.get_depth_frame().apply_filter(color_map);
		rs2::video_stream_profile depthfile(aligned_depth_frame.get_profile());
		rs2_intrinsics depth_intrin = depthfile.get_intrinsics();
		const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
		const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
		const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
		const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
		if (!aligned_depth_frame || !aligned_color_frame) continue;
		cv::Mat aligned_depth_image(cv::Size(depth_w, depth_h), CV_8UC3, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat aligned_color_image(cv::Size(color_w, color_h), CV_8UC3, (void*)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
		std::vector<cv::RotatedRect> lightbar = process(aligned_color_image);
		if (lightbar.size() >= 2) {
			cv::line(bkg, cv::Point(250, 500), cv::Point(0, 0), cv::Scalar(100, 255, 100), 3);
			cv::line(bkg, cv::Point(250, 500), cv::Point(500, 0), cv::Scalar(100, 255, 100), 3);
			std::vector<ARMOR> armor = distinguish(lightbar);
			getinfo(aligned_color_image, armor);
			pixiv_to_point(armor, aligned_depth_frame_ori, depth_intrin);
			visualization(color_win, aligned_color_image, depth_win, aligned_depth_image, lightbar, armor);
		}
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error& e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

//////////////////////////////////////////////
float get_depth_scale(rs2::device dev) {
	for (rs2::sensor& sensor : dev.query_sensors())
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) return dpt.get_depth_scale();
	throw std::runtime_error("Device does not have a depth sensor");
}
bool JUI(cv::RotatedRect A, cv::RotatedRect B) {
	if (abs(A.angle - B.angle) > 10)
		return 0;
	if ((A.size.height + B.size.height) / 2 * DIS(A.center, B.center) < 100)
		return 0;

	if (abs((acos((A.center.y - B.center.y) / DIS(A.center, B.center)) * 57.3) - 90) > 35)
		return 0;
	if (DIS(A.center, B.center) / (A.size.width + A.size.height + B.size.width + B.size.height) > 1.2)
		return 0;
	if (abs(((abs(A.angle) + abs(B.angle)) / 2 + acos((A.center.y - B.center.y) / DIS(A.center, B.center)) * 57.3) - 90) > 30)
		return 0;

	if (abs(A.size.height - B.size.height) > (A.size.height + B.size.height) / 5)
		return 0;
	if (abs(A.center.y - B.center.y) > (A.size.height + B.size.height) / 10)
		return 0;
	if (abs(A.size.width / A.size.height - B.size.width / B.size.height) > 2)
		return 0;

	return 1;
}
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev) {
	for (auto&& sp : prev) {
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) return true;
	}
	return false;
}

std::vector<cv::RotatedRect> process(cv::Mat src_ori) {
	cv::Mat src, src_hsv, src_bi, src_op, src_cn, src_ps;
	src = src_ori(cv::Rect(140, 250, 1000, 450));
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 2));
	cv::cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);
	std::vector<cv::Mat> BGRchannel;
	cv::split(src_hsv, BGRchannel);
	cv::threshold(BGRchannel.at(2), src_bi, 250, 255, cv::THRESH_BINARY);
	cv::morphologyEx(src_bi, src_op, cv::MORPH_OPEN, element);
	cv::Canny(src_op, src_cn, 25, 75);
	cv::dilate(src_cn, src_ps, kernel);

	std::vector<cv::RotatedRect> lightbar;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(src_ps, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); i++)
		if (cv::contourArea(contours[i]) > 30 && contours[i].size() > 6)
			lightbar.push_back(cv::fitEllipse(contours[i]));
	if (lightbar.size() >= 2)
		for (size_t i = 0; i < lightbar.size(); i++)
			if (lightbar[i].angle >= 90)
				lightbar[i].angle = lightbar[i].angle - 180;
	return lightbar;
}
std::vector<ARMOR> distinguish(std::vector<cv::RotatedRect> lightbar) {
	std::vector<ARMOR> armor;
	for (size_t i = 0; i < lightbar.size() - 1; i++)
		for (size_t t = i + 1; t < lightbar.size(); t++)
			if (JUI(lightbar[i], lightbar[t])) {
				ARMOR armor_this;
				armor_this.center = (lightbar[i].center + lightbar[t].center) / 2;
				armor_this.angle = (lightbar[i].angle + lightbar[t].angle) / 114.6;
				armor_this.pt[0] = lightbar[i].center + cv::Point2f(lightbar[i].size.height * sin(armor_this.angle), -lightbar[i].size.height * cos(armor_this.angle)) * 0.8;
				armor_this.pt[1] = lightbar[i].center - cv::Point2f(lightbar[i].size.height * sin(armor_this.angle), -lightbar[i].size.height * cos(armor_this.angle)) * 0.8;
				armor_this.pt[3] = lightbar[t].center + cv::Point2f(lightbar[t].size.height * sin(armor_this.angle), -lightbar[t].size.height * cos(armor_this.angle)) * 0.8;
				armor_this.pt[2] = lightbar[t].center - cv::Point2f(lightbar[t].size.height * sin(armor_this.angle), -lightbar[t].size.height * cos(armor_this.angle)) * 0.8;
				armor.push_back(armor_this);
			}
	return armor;
}
void getinfo(cv::Mat src_color, std::vector<ARMOR>& armor) {
	cv::Point2f dst[4] = { {128.0f, 0.0f}, {128.0f, 128.0f}, {0.0f, 128.0f}, {0.0f, 0.0f} };
	cv::Mat transform, transform_hsv;
	for (size_t i = 0; i < armor.size(); i++) {
		cv::warpPerspective(src_color, transform, cv::getPerspectiveTransform(armor[i].pt, dst), cv::Point(128, 128));
		int R = 0, B = 0;
		for (size_t k = 0; k < 15; k++)
			for (size_t t = 0; t < 3; t++) {
				B += transform.at<cv::Vec3b>(57 + k, t)[0];
				B += transform.at<cv::Vec3b>(57 + k, 127 - t)[0];
				R += transform.at<cv::Vec3b>(57 + k, t)[2];
				R += transform.at<cv::Vec3b>(57 + k, 127 - t)[2];
			}
		B = B / 90;
		R = R / 90;
		if (B - R > 20 && B > 220)
			armor[i].camp = 0;
		else if (R - B > 20 && R > 220)
			armor[i].camp = 1;
		armor[i].center = cv::Point(armor[i].center.x + 140, armor[i].center.y + 250);
		//cv::imshow(std::to_string(i), transform);
	}
}
void pixiv_to_point(std::vector<ARMOR>& armor, rs2::depth_frame aligned_depth_frame, rs2_intrinsics depth_intrin) {
	for (size_t i = 0; i < armor.size(); i++) {
		float pixiv[2] = { armor[i].center.x, armor[i].center.y };
		float dis = aligned_depth_frame.get_distance((int)pixiv[0], (int)pixiv[1]);
		rs2_deproject_pixel_to_point(armor[i].point, &depth_intrin, pixiv, dis);
	}
}
void visualization(const char* color_win_name, cv::Mat src_color, const char* depth_win_name, cv::Mat src_depth, std::vector<cv::RotatedRect> lightbar, std::vector<ARMOR> armor) {
	cv::Mat src = src_color(cv::Rect(140, 250, 1000, 450));
	for (size_t i = 0; i < lightbar.size(); i++) {
		cv::Point2f pt[4];
		lightbar[i].points(pt);
		for (size_t k = 0; k < 4; k++)
			cv::line(src, pt[k], pt[(k + 1) % 4], cv::Scalar(150, 250, 80), 1);
	}
	for (size_t i = 0; i < armor.size(); i++) {
		cv::circle(src_depth, cv::Point(armor[i].center.x, armor[i].center.y), 5, cv::Scalar(250, 80, 255), cv::FILLED);
		cv::circle(bkg, cv::Point(armor[i].point[0] * 100 + 250, 500 - armor[i].point[2] * 100), 5, cv::Scalar(250, 80, 255), cv::FILLED);
		if (armor[i].camp == 0)
			for (size_t k = 0; k < 4; k++)
				cv::line(src, armor[i].pt[k], armor[i].pt[(k + 1) % 4], cv::Scalar(255, 145, 125), 2);
		else if (armor[i].camp == 1) {
			for (size_t k = 0; k < 4; k++)
				cv::line(src_color, armor[i].pt[k], armor[i].pt[(k + 1) % 4], cv::Scalar(90, 125, 255), 2);
			cv::circle(src, armor[i].center, 5, cv::Scalar(50, 80, 255), cv::FILLED);
		}
		else
			for (size_t k = 0; k < 4; k++)
				cv::line(src, armor[i].pt[k], armor[i].pt[(k + 1) % 4], cv::Scalar(255, 0, 240), 2);
	}
	cv::imshow(color_win_name, src);
	//cv::imshow(depth_win_name, src_depth);
	cv::imshow(depth_win_name, bkg);
}

//////////////////////////////////////////////

