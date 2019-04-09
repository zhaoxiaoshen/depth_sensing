#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
//using namespace cv;
using namespace std;

#define CV_EVENT_FLAG_LBUTTON 1           //�����ק  
#define CV_EVENT_FLAG_RBUTTON 2           //�Ҽ���ק  
#define CV_EVENT_FLAG_MBUTTON 4           //�м���ק  
#define CV_EVENT_FLAG_CTRLKEY 8     //(8~15)��Ctrl�����¼�  
#define CV_EVENT_FLAG_SHIFTKEY 16   //(16~31)��Shift�����¼�  
#define CV_EVENT_FLAG_ALTKEY 32      // (32~39)��Alt�����¼�

typedef struct matchInfoSt {
	cv::Mat srcImg;
	vector<cv::Mat> templateImg;
	vector<cv::Mat> secondTemplateImg;
	cv::Rect roiSet;
	cv::Rect roiFind;
	std::vector<cv::Rect> multiRoi;
}matchInfoSt;
 
static cv::Mat srcF,tmpF;
void on_mouse(int event, int x, int y, int flags, void *ustc);	

class lockFinder
{
public:
	lockFinder();
	~lockFinder();
public:
	//void on_mouse(int event, int x, int y, int flags, void *ustc);
	int matchMethod;
public:
	void roiSet(cv::Mat imgSrc, cv::Rect& roi);
	void onMatch(int func, void *info, cv::Rect roiSet, cv::Rect &roiFind, std::vector<cv::Mat> templateVec);
	void on_Matching(int, void* info, std::vector<cv::Mat>& imgPro);
	void lockProcess(cv::Mat img, cv::Rect& rect, std::vector<cv::Mat>&imgPro);
	
};

