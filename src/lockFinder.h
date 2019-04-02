#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
//using namespace cv;
using namespace std;

#define CV_EVENT_FLAG_LBUTTON 1           //左键拖拽  
#define CV_EVENT_FLAG_RBUTTON 2           //右键拖拽  
#define CV_EVENT_FLAG_MBUTTON 4           //中间拖拽  
#define CV_EVENT_FLAG_CTRLKEY 8     //(8~15)按Ctrl不放事件  
#define CV_EVENT_FLAG_SHIFTKEY 16   //(16~31)按Shift不放事件  
#define CV_EVENT_FLAG_ALTKEY 32      // (32~39)按Alt不放事件

typedef struct matchInfoSt {
	cv::Mat srcImg;
	vector<cv::Mat> templateImg;
	cv::Rect roiSet;
	cv::Rect roiFind;
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
	void on_Matching(int, void* info, std::vector<cv::Mat>& imgPro);
	void lockProcess(cv::Mat img, cv::Rect& rect, std::vector<cv::Mat>&imgPro);
	
};

