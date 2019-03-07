#include "lockFinder.h"



lockFinder::lockFinder()
{
}


lockFinder::~lockFinder()
{
}

void lockFinder::roiSet(cv::Mat imgSrc, cv::Rect& roi)
{
	imgSrc.copyTo(srcF);
	imgSrc.copyTo(tmpF);
	cv::namedWindow("img",0);
	cv::resizeWindow("img", 1104, 621);
	cv::setMouseCallback("img", on_mouse, &roi);//���ûص�����  
	cv::imshow("img", srcF);
	cv::waitKey(2);
	while (1)
	{
		if (cv::waitKey(1000) >= 0)
		{
			cv::destroyWindow("img");
			break;
		}
	}
}
void on_mouse(int event, int x, int y, int flags, void *roi)
{
	cv::Rect* roiRect = (cv::Rect*)roi;
	static cv::Point pre_pt(-1.0, -1.0);//��ʼ����  
	static cv::Point cur_pt(-1, -1);//ʵʱ����  
	char temp[16];
	cv::Mat img = srcF;
	if (event == CV_EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
	{
		srcF.copyTo(img);//��ԭʼͼƬ���Ƶ�img��  
		//sprintf_s(temp, "(%d,%d)", x, y);
		sprintf(temp, "(%d,%d)", x, y);
		pre_pt = cv::Point(x, y);
		roiRect->x = pre_pt.x;
		roiRect->y = pre_pt.y;
		cv::putText(img, temp, pre_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255), 1, 8);//�ڴ�������ʾ����  
		cv::circle(img, pre_pt, 2, cv::Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//��Բ  
		cv::imshow("img", img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//���û�а��µ����������ƶ��Ĵ�����  
	{
		img.copyTo(tmpF);//��img���Ƶ���ʱͼ��tmpF�ϣ�������ʾʵʱ����  
		//sprintf_s(temp, "(%d,%d)", x, y);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		cv::putText(tmpF, temp, cur_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));//ֻ��ʵʱ��ʾ����ƶ�������  
		cv::imshow("img", tmpF);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//�������ʱ������ƶ�������ͼ���ϻ�����  
	{
		img.copyTo(tmpF);
		//sprintf_s(temp, "(%d,%d)", x, y);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		//roiRect->x = x;
		//roiRect->y = y;
		cv::putText(tmpF, temp, cur_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));
		rectangle(tmpF, pre_pt, cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);//����ʱͼ����ʵʱ��ʾ����϶�ʱ�γɵľ���  
		cv::imshow("img", tmpF);
	}
	else if (event == CV_EVENT_LBUTTONUP)//����ɿ�������ͼ���ϻ�����  
	{
		srcF.copyTo(img);
		//sprintf_s(temp, "(%d,%d)", x, y);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		roiRect->height = (y - roiRect->y);
		roiRect->width = (x - roiRect->x);
		cv::putText(img, temp, cur_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));
		circle(img, pre_pt, 2, cv::Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cv::rectangle(img, pre_pt, cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);//���ݳ�ʼ��ͽ����㣬�����λ���img��  
		imshow("img", img);
	
		//��ȡ���ΰ�Χ��ͼ�񣬲����浽dst��  
		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			printf("width == 0 || height == 0");
			return;
		}
		cv::Mat dst = srcF(cv::Rect(cv::min(cur_pt.x, pre_pt.x), cv::min(cur_pt.y, pre_pt.y), width, height));
		//cv::namedWindow("dst");
		//cv::imshow("dst", dst);
		//cv::waitKey(0);
	}
}

void lockFinder::on_Matching(int, void* info,std::vector<cv::Mat>& imgPro)
{
	matchInfoSt* matchInfo = (matchInfoSt*)info;
	//��1�����ֲ�������ʼ��
	//Mat srcImage;
	//g_srcImage.copyTo(srcImage);
	cv::Mat srcImageS, srcImage;
	matchInfo->srcImg.copyTo(srcImage);
	matchInfo->srcImg.copyTo(srcImageS);
	cv::Mat templateImage;
	matchInfo->templateImg.copyTo(templateImage);
	
	cv::Mat imageShowRoi;
	srcImage.copyTo(imageShowRoi);
	cv::rectangle(imageShowRoi, cv::Point(matchInfo->roiSet.x, matchInfo->roiSet.y),
		cv::Point(matchInfo->roiSet.x + matchInfo->roiSet.width, matchInfo->roiSet.y + matchInfo->roiSet.height),
		cv::Scalar(0, 0, 255), 2, 8, 0);
	//cv::imwrite("roi_set.jpg",imageShowRoi);
	
	srcImage = srcImage(matchInfo->roiSet);
	cv::imwrite("roi_imag.jpg",srcImage);
	//��2����ʼ�����ڽ������ľ���
	cv::Mat g_resultImage;
	int resultImage_cols = srcImage.cols - templateImage.cols + 1;
	int resultImage_rows = srcImage.rows - templateImage.rows + 1;
	if(resultImage_cols<=0||resultImage_rows<=0)
	{
		printf("roi set error or template image is too big!\n");
		return;
	}
	g_resultImage.create(resultImage_cols, resultImage_rows, CV_32FC1);

	//��3������ƥ��ͱ�׼��
	cv::cvtColor(srcImage, srcImage, CV_RGB2GRAY);
	cv::cvtColor(templateImage, templateImage, CV_RGB2GRAY);
	cv::equalizeHist(srcImage, srcImage);
	equalizeHist(templateImage, templateImage);
	matchTemplate(srcImage, templateImage, g_resultImage, matchMethod);
	cv::normalize(g_resultImage, g_resultImage, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	//��4��ͨ������ minMaxLoc ��λ��ƥ���λ��
	double minValue; double maxValue; cv::Point minLocation; cv::Point maxLocation;
	cv::Point matchLocation;
	cv::minMaxLoc(g_resultImage, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());

	//��5�����ڷ��� SQDIFF �� SQDIFF_NORMED, ԽС����ֵ���Ÿ��ߵ�ƥ����. ������ķ���, ��ֵԽ��ƥ��Ч��Խ��
	//�˾�����OpenCV2��Ϊ��
	//if( g_nMatchMethod  == CV_TM_SQDIFF || g_nMatchMethod == CV_TM_SQDIFF_NORMED )
	//�˾�����OpenCV3��Ϊ��
	if (matchMethod == CV_TM_SQDIFF || matchMethod == CV_TM_SQDIFF_NORMED)
	{
		matchLocation = minLocation;
	}
	else
	{
		matchLocation = maxLocation;
	}

	//��6�����Ƴ����Σ�����ʾ���ս��
	cv::rectangle(srcImage, matchLocation, cv::Point(matchLocation.x + templateImage.cols, matchLocation.y + templateImage.rows), cv::Scalar(0, 0, 255), 2, 8, 0);
	cv::rectangle(g_resultImage, matchLocation, cv::Point(matchLocation.x + templateImage.cols, matchLocation.y + templateImage.rows), cv::Scalar(0, 0, 255), 2, 8, 0);

	cv::imwrite("match-src.jpg",srcImage);
	//cv::imshow("match-src", srcImage);
	//cv::waitKey(2);

	//cv::imshow("match-result", g_resultImage);
	//cv::waitKey(2);

	//====
	cv::Rect lockRoi;
	lockRoi.x = matchLocation.x;
	lockRoi.y = matchLocation.y;
	lockRoi.width = templateImage.cols;
	lockRoi.height = templateImage.rows;
	matchInfo->roiFind = lockRoi;
	//== proccess
	cv::Mat lockB;
	srcImageS.copyTo(lockB);
	lockB = lockB(matchInfo->roiSet);
	lockRoi.x -= 70; lockRoi.y -= 50; lockRoi.height += 150; lockRoi.width += 150;
	lockRoi.x = lockRoi.x > 0 ? lockRoi.x : 1; lockRoi.y = lockRoi.y > 0 ? lockRoi.y : 1; 
	lockRoi.height = lockRoi.height < (lockB.rows - lockRoi.y) ? lockRoi.height : (lockB.rows - lockRoi.y);
	lockRoi.width = lockRoi.width < (lockB.cols - lockRoi.x) ? lockRoi.width : (lockB.cols - lockRoi.x);
	lockRoi.width = lockRoi.width > 0 ? lockRoi.width : 0;
	lockRoi.height = lockRoi.height > 0 ? lockRoi.height : 0;
	lockB = lockB(lockRoi);
	lockProcess(lockB, lockRoi,imgPro);
}

void lockFinder::lockProcess(cv::Mat img, cv::Rect& rect,std::vector<cv::Mat>&imgPro)
{
	//����ͨ��
	//cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	//cv::Mat channels[3];
	//split(img, channels);
	//cv::imshow("RED", channels[2]);
	//cv::imwrite("red.jpg", channels[2]);
	//cv::waitKey(2);

	//convert to gray
	cv::Mat imgGray;
	cv::cvtColor(img, imgGray, CV_BGR2GRAY);
	//cv::imshow("gray", imgGray);
	//cv::imwrite("gray.jpg", imgGray);
	//cv::waitKey(2);
	imgPro.push_back(imgGray);

	// adaptiveThreshold
	int blockSize = 13;
	int constValue = 15;
	cv::Mat thresholdLocal;
	cv::adaptiveThreshold(imgGray, thresholdLocal, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, blockSize, constValue);
	//cv::imshow("threshold", thresholdLocal);
	//cv::imwrite("threshold.jpg", thresholdLocal);
	//cv::waitKey(2);
	imgPro.push_back(thresholdLocal);

	//canny 
	cv::Mat cannyImg;
	cv::Canny(imgGray, cannyImg, 80, 255);
	//cv::Canny(cv::imread("grayFilter.jpg"), cannyImg,100, 255);
	
	//cv::imshow("canny", cannyImg);
	//cv::imwrite("canny.jpg", cannyImg);
	//cv::waitKey(2);
	imgPro.push_back(cannyImg);

	//==find contours
	cv::Mat show = img;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	//std::vector<cv::Rect> rectC;
	cv::findContours(thresholdLocal, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	int idx = 0;
	for (; idx < contours.size(); idx++)
	{
		cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
		cv::drawContours(show, contours, idx, color, CV_FILLED, 8);
		//rect.push_back(cv::boundingRect(contours));
		//cv::rectangle(dst,cv::boundingRect(contours.at(idx)),color);

	}
	//cv::imshow("contors", show);
	//cv::imwrite("contours.jpg", show);
	//cv::waitKey(2);
	imgPro.push_back(show);
}
