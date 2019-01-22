#pragma once
#include <sl/Camera.hpp>

#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
//#include "utils.hpp"
#include "lockFinder.h"
using namespace sl;
using namespace std;
class zedImage
{
public:
	zedImage();
	~zedImage();
public:
	Camera zed;
	Resolution imageSize;
	int width, height, width_sbs;
	RuntimeParameters rt_param;
	sl::Mat left_image,right_image,depth_image;
	//==playback
	int nb_frames,svo_positoion;

	//===match
	lockFinder Finder;
	matchInfoSt matchInfo;
	cv::Mat srcImage, templateImage;

	//==mesure
	
public:
	void init();
	int zedOpen(InitParameters initParameters);
	int zedImageGet(cv::Mat& image,int type);
	cv::Mat slMat2cvMat(sl::Mat &input);
	void roiSelect(cv::Mat img);
	void roiSelect(cv::Mat img, float x, float y, float height, float width);

	int templateImageLoad(std::string imageName);
	int zed_match(std::vector<cv::Mat>& imgPro);
	int measure(sl::float4& point3D,cv::Mat image);

};

