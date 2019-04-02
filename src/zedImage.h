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

typedef struct detectInfoSt{

	int  nInstalledPos;	// ��װλ�ã�1Ϊ���࣬0Ϊ½��	
	float roi_x;
	float roi_y;
	float roi_width;
	float roi_height; //ͼ��ٷֱ�
	int camera_sn;

	std::vector<std::string> templatePath;
	std::string videoReadPath;
	float result[3]; //cloudPoint xyz
	float calibratePos[3];//У׼λ�� xyz
	int db;
	int address_r;
	int offset_r;
	int len_r;
	int address_w;
	int offset_w;
	int len_w;
	float distance[3];
	int nDetectStep;
	/* bool cameraStatus;*/
	int resolution;
	int address_s; //����״̬
	int offset_s;
}detectInfoSt;


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
	detectInfoSt detectInfo;
	bool getCameraStatus(){ return cameraStatus; };
private:
	bool cameraStatus;
	//==mesure
	
public:
	void init();
	int zedOpen(InitParameters initParameters);
	int zedClose();
	int zedImageGet(cv::Mat& image,int type);
	cv::Mat slMat2cvMat(sl::Mat &input);
	void roiSelect(cv::Mat img);
	void roiSelect(cv::Mat img, float x, float y, float height, float width);

	int templateImageLoad(/*std::vector<std::string> imageName*/);
	int zed_match(std::vector<cv::Mat>& imgPro);
	int measure(sl::float4& point3D,cv::Mat image);

};

