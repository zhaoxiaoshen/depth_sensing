

// Standard includes
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"

#include "zedImage.h"

#include "iniRead.h"
#include <vector>
#include <time.h>
#include <unistd.h>
#include <atomic>
// plc connect 
#include "plcConnect.h"

#define CAM_RUNING_MAX_NUM    2

#define _X		0
#define _Y		1
#define _Z		2
#define SeaSide		1
#define LandSide	0

// Using std and sl namespaces
using namespace std;
using namespace sl;
//using namespace cv;

// Create ZED objects (camera, callback, images)
sl::Camera zed;
sl::Mat point_cloud;
std::thread zed_callback;
int width, height;
bool quit;
std::atomic<int> cameraRunNum (0);

// Point cloud viewer
//GLViewer viewer;

float m_fResultX;
float m_fResultY;
float m_fResultZ;

float m_fResultXLive;
float m_fResultYLive;
float m_fResultZLive;

struct CoordinateBuffer
{
	unsigned char 	nXDirection;
	unsigned char	nXValue;
	unsigned char	nYDirection;
	unsigned char	nYValue;
	unsigned char	nZDirection;
	unsigned char	nZValue;
};

// Sample functions
void startZED();
void *run(void *arg);
void close();

void windowInit();
void cameraOpen(zedImage *processor, int cameraIndex);
void cameraClose();
#define EnablePLC	(1)

static void stringSplit(const std::string& src, const std::string& separator, std::vector<std::string>& dest)
{
	dest.clear();
	std::string str = src;
	std::string substring;
	std::string::size_type start = 0, index;
	do
	{
		index = str.find_first_of(separator,start);
		if (index != std::string::npos)
		{    
			substring = str.substr(start,index-start);
			dest.push_back(substring);
			start = str.find_first_not_of(separator,index);
			if (start == std::string::npos) return;
		}
	}while(index != std::string::npos);
	//the last token
	substring = str.substr(start);
	dest.push_back(substring);
}

static void  configInit(detectInfoSt& detectInfo, CIni& iniFile)
{
	printf("%s\n", __PRETTY_FUNCTION__);
	detectInfo.camera_sn = iniFile.GetInt("CAMERA", "sn");
	printf("Camera SN = %d.\n", detectInfo.camera_sn);
    detectInfo.method = iniFile.GetInt("ALGO_SELECT", "algo");
    printf("algo select: %d\n", detectInfo.method);
    detectInfo.roi_x = iniFile.GetFloat("ROI_SET", "x");
    detectInfo.roi_y = iniFile.GetFloat("ROI_SET", "y");
    detectInfo.roi_height = iniFile.GetFloat("ROI_SET", "height");
    detectInfo.roi_width = iniFile.GetFloat("ROI_SET", "width");
	detectInfo.nDetectStep = iniFile.GetInt("ROI_SET", "Step");
	printf("%s detect step = %d \r\n", __PRETTY_FUNCTION__, detectInfo.nDetectStep);
    //detectInfo.templatePath = iniFile.GetStr("TEMPLATE_IMAGE", "path");
	iniFile.GetSection("TEMPLATE_IMAGE", detectInfo.templatePath);
	printf("Template Array Size = %d.\n",(int)detectInfo.templatePath.size());
    for (unsigned int i = 0; i < detectInfo.templatePath.size(); i++)
    {
        printf("template image: %s \n", detectInfo.templatePath[i].c_str());
    }
    if (detectInfo.method == MATCH_TWICE)
    {
        iniFile.GetSection("SECOND_TEMPLATE_IMAGE", detectInfo.secondTemplatePath);
        printf("Template Array Size = %d.\n", (int)detectInfo.secondTemplatePath.size());
        for (unsigned int i = 0; i < detectInfo.secondTemplatePath.size(); i++)
        {
            printf("template image: %s \n", detectInfo.secondTemplatePath[i].c_str());
        }
    }
    //detectInfo.camera_sn = 12224;
	//detectInfo.camera_sn = 18711;
	//detectInfo.camera_sn = 21057;
	//detectInfo.camera_sn = 21098;
   
	detectInfo.nInstalledPos = iniFile.GetInt("CAMERA", "InstalledPosition");
	printf("InstalledPosition = %s\r\n",(detectInfo.nInstalledPos == 1)?"seaside":"landside");
	
    detectInfo.videoReadPath = iniFile.GetStr("VIDEO","read_path");
    printf("config read roi x:%f y:%f width:%f height:%f \n camera sn:%d\n video read:%s\n",
        detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_width, detectInfo.roi_height,
        detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
    detectInfo.db = iniFile.GetInt("DB_READ_SET", "db");
    detectInfo.address_r = iniFile.GetInt("DB_READ_SET", "address");  
    detectInfo.address_w = iniFile.GetInt("DB_RESULT_SET", "address");
    detectInfo.offset_r = iniFile.GetInt("DB_READ_SET", "offset");
    detectInfo.resolution = iniFile.GetInt("IMAGE_RESOLUTION","resolution");
    printf ("resolution:%d \n", detectInfo.resolution);
    printf("db:%d address read:%d offset:%d address write:%d \n",
            detectInfo.db, detectInfo.address_r, detectInfo.offset_r,  detectInfo.address_w);
    detectInfo.address_s = iniFile.GetInt("DB_WRITE_SET", "address");
    detectInfo.offset_s = iniFile.GetInt("DB_WRITE_SET", "offset");
    printf("db:%d address status write:%d offset:%d \n",
            detectInfo.db, detectInfo.address_s, detectInfo.offset_s);
    detectInfo.calibratePos[_X] = iniFile.GetFloat("CALIBRATION_POS", "x");
    detectInfo.calibratePos[_Y] = iniFile.GetFloat("CALIBRATION_POS", "y");
    detectInfo.calibratePos[_Z] = iniFile.GetFloat("CALIBRATION_POS", "z");
    printf("calibration offset x: %f y: %f z: %f \n",
        detectInfo.calibratePos[_X], detectInfo.calibratePos[_Y], detectInfo.calibratePos[_Z]);
}
 
std::string getTime(void)
{
    char time_str[255]={0};
    struct timeval tv;
    struct timezone tz;   
    struct tm *tmNow;
    gettimeofday(&tv, &tz);
    tmNow = localtime(&tv.tv_sec);
    snprintf(time_str,sizeof(time_str)-1,"%04d_%02d_%02d-%02d_%02d_%02d_%03ld",
        tmNow->tm_year + 1900, tmNow->tm_mon + 1, tmNow->tm_mday, tmNow->tm_hour, tmNow->tm_min,
        tmNow->tm_sec, tv.tv_usec/1000);
    
    return time_str;
}

std::vector<detectInfoSt> detectInfoVec;
std::vector<zedImage*> zedProcessorVec;

void cameraClose(int index)
{
    printf("%s camra close index:%d \n", getTime().c_str(), index);
    zedProcessorVec[index]->zedClose();
    //sleep(1);
    //detectInfoVec[index].cameraStatus = false;
    cameraRunNum --;
}

void *cameraProcess(void *argv)
{
    int cameraIndex = *(int *)argv;
	zedImage* zedProcessor;
    zedProcessor = zedProcessorVec[cameraIndex];
	detectInfoSt &detectInfo = zedProcessor->detectInfo;
   
    int cameraRunningCount = cameraRunNum;
    printf("\n%s camera running index %d runNum:%d \n", getTime().c_str(),
        cameraIndex, cameraRunningCount);
	if (!zedProcessor->getCameraStatus())
	{
        cameraOpen(zedProcessor, cameraIndex); //"" for live
        usleep(500*1000); //500ms
    }
    cv::Mat image, imageR;
    sl::float4 pointResult;
    pointResult.x = 0;
    pointResult.y = 0;
    pointResult.z = 0;
    if (zedProcessor->zedImageGet(image, VIEW_LEFT))
    {
        printf("can not read image \n");
        cameraClose(cameraIndex);
        return NULL;
    }
    if (!image.empty())
    {
        image.copyTo(imageR);
    }
    else
    {
		printf("%s Get a empty image.X X X X X X.\r\n", __PRETTY_FUNCTION__);
        cameraClose(cameraIndex);
        return NULL;
    }
    std::vector<cv::Mat> imgPro;

    printf("image match begin \n");
    zedProcessor->zed_match(imgPro);
    zedProcessor->measure(pointResult, imageR);

	char srcNameSave[512] = {0};
    char dstNameSave[512] = {0};
	snprintf(srcNameSave, sizeof(srcNameSave)-1, "img/%s_%d_src.jpg",
        getTime().c_str(), detectInfo.camera_sn);
	snprintf(dstNameSave, sizeof(dstNameSave)-1, "img/%s_%d_dst.jpg",
		getTime().c_str(), detectInfo.camera_sn);
	cv::imwrite(srcNameSave, imageR);
	
    cv::cvtColor(imageR, imageR, CV_BGR2GRAY);
    printf("image size cols %d rows %d\n", imageR.cols, imageR.rows);
    image.release();
    imageR.release();

    detectInfo.distance[_X] = pointResult.x - detectInfo.calibratePos[_X];
    detectInfo.distance[_Y] = pointResult.y - detectInfo.calibratePos[_Y];
    detectInfo.distance[_Z] = pointResult.z - detectInfo.calibratePos[_Z];
    printf("%s: index:%d point cloud x %f  y %f  z %f \n", getTime().c_str(), cameraIndex, pointResult.x, pointResult.y, pointResult.z);
	printf("%s: index:%d defference of distance:x %f  y %f  z %f \n", getTime().c_str(), cameraIndex, detectInfo.distance[_X], detectInfo.distance[_Y], detectInfo.distance[_Z]);
    //delete (int *)argv;
    //== write result
    unsigned char value = 0x01;
    unsigned char buffer[255] = {0};
	
	CoordinateBuffer Result;
	Result.nYDirection = detectInfo.distance[_Y] > 0 ? 1 : 0;
	Result.nYValue = (unsigned char)abs(detectInfo.distance[_Y]);
	if (!Result.nYValue)
		Result.nYValue = 1;

	if (detectInfo.nInstalledPos == SeaSide)
	{
		// 面向海侧右为正
		Result.nXDirection = detectInfo.distance[_X] > 0 ? 0 : 1;
		Result.nXValue = (unsigned char)abs(detectInfo.distance[_X]);
		if (!Result.nXValue)		//若差量为0，则给个最小值
			Result.nXValue = 1;

		// 面向海侧前为正
		Result.nZDirection = detectInfo.distance[_Z] > 0 ? 0 : 1;
		Result.nZValue = (unsigned char)abs(detectInfo.distance[_Z]);
		if (!Result.nZValue)
			Result.nZValue = 1;
	}
	else
	{
		// 面向海侧右为正
		Result.nXDirection = detectInfo.distance[_X] > 0 ? 1 : 0;
		Result.nXValue = (unsigned char)abs(detectInfo.distance[_X]);
		if (!Result.nXValue)		//若差量为0，则给个最小值
			Result.nXValue = 1;

		// 面向海侧前为正
		Result.nZDirection = detectInfo.distance[_Z] > 0 ? 1 : 0;
		Result.nZValue = (unsigned char)abs(detectInfo.distance[_Z]);
		if (!Result.nZValue)
			Result.nZValue = 1;

	}
	printf("X Direction=%s\tValue = %d.\n", (Result.nXDirection == 1) ? "Rightside" : "Leftside",Result.nXValue);
	printf("Y Direction=%s\tValue = %d.\n", (Result.nYDirection == 1) ? "Down" : "Up",Result.nYValue);
	printf("Z Direction=%s\tValue = %d.\n", (Result.nZDirection == 1) ? "SeaSide" : "Lanside",Result.nZValue);
//     buffer[0] = ((int)(pointResult.x) & 0xff00) >> 8;
//     buffer[1] = ((int)(pointResult.x) & 0x00ff);
//     buffer[2] = ((int)(pointResult.y) & 0xff00) >> 8;
//     buffer[3] = ((int)(pointResult.y) & 0x00ff);
//     buffer[4] = ((int)(pointResult.z) & 0xff00) >> 8;
//     buffer[5] = ((int)(pointResult.z) & 0x00ff);

	dataWrite(detectInfo.db, detectInfo.address_w, 6, &Result);

    int nRes = dataRead(detectInfo.db, detectInfo.address_s, 1, buffer);
    if (!nRes)
    {
        value = 0x01;
        value <<= detectInfo.offset_s;
        buffer[0] |= value;
        dataWrite(detectInfo.db, detectInfo.address_s, 1, buffer);
    }

    sl::sleep_ms(10);
    //cameraClose(cameraIndex);
    //printf("%s camera stop index %d\n", getTime().c_str(), cameraIndex);
	printf("%s returned.\r\b", __PRETTY_FUNCTION__);
}

void zedWorkStart(int index)
{
    pthread_t tidp;
   // int *cameraIndex = new int(0);
    int cameraIndex = index;
    if ((pthread_create(&tidp, NULL, &cameraProcess, (void *)&cameraIndex) == -1))
    {
        printf("create zed work error!\n");
        return;
    }
    pthread_detach(tidp);
    printf("camera %d start ....\n", index);
    usleep(5000);
}

void mainCamKeepLive()
{
    int cameraOpenNum = zedProcessorVec.size();
    cameraOpenNum = cameraOpenNum > 2 ? 2 : cameraOpenNum;
    for (int cameraIndex = 0; cameraIndex < cameraOpenNum; cameraIndex++)
    {
		if (!zedProcessorVec[cameraIndex]->getCameraStatus())
        {
            cameraOpen(zedProcessorVec[cameraIndex], cameraIndex);
        }
        else
        {
            cv::Mat image;
            if (zedProcessorVec[cameraIndex]->zedImageGet(image, VIEW_LEFT))
            {
                printf("image update fail\n");
            }
            image.release();
        }
    }
}

int main(int argc, char **argv)
{
    // Start the camera thread

    //==read config.ini
    int cameraNum = 0;
    std::string plcIp = "127.0.0.1";

    CIni iniFile;
	iniFile.EnableOuput(false);
    if (iniFile.OpenFile("config/General.ini") != INI_SUCCESS)
    {
        printf("read config file General.ini failed!\n");
        return -1;
    }
	plcIp = iniFile.GetStr("PLC", "ip");
	cameraNum = iniFile.GetInt("CAMERA", "num");
    printf("camera num:%d plc ip:%s \n", cameraNum, plcIp.c_str());
	if (cameraNum > 0)
	{
		std::vector<std::string> vecCameraConfig;
		iniFile.GetSection("Config", vecCameraConfig);
		for (int i = 0; i < cameraNum; i++)
		{
			detectInfoSt detectInfo;
			char path[255] = {0};
			snprintf(path, sizeof(path) - 1, "config/%s", vecCameraConfig[i].c_str());
			printf("Camera config file:%s.\n", path);
			//iniFile.EnableOuput(true);
			if (iniFile.OpenFile(path) != INI_SUCCESS)
			{
				printf("read config file %s failed!\n", path);
				return -1;
			}
			else
			{
				zedImage* zedProcessor = new zedImage();
				configInit(zedProcessor->detectInfo, iniFile);
                zedProcessor->templateImageLoad(zedProcessor->detectInfo.templatePath,
                    zedProcessor->matchInfo.templateImg);
                if (zedProcessor->detectInfo.method == 2)
                {
                    zedProcessor->templateImageLoad(zedProcessor->detectInfo.secondTemplatePath,
                    zedProcessor->matchInfo.secondTemplateImg);
                }
				//zedProcessor->templateImageLoad(/*detectInfo.templatePath*/);
				//cameraOpen(*zedProcessor, i); //"" for live
				zedProcessorVec.push_back(zedProcessor);
				zedProcessorVec[i]->zedClose();
			}
		}
	}

    // windowInit();
	
	plcConnect(plcIp);

    quit = false;
    unsigned int idleTime = 0;
    mainCamKeepLive();
    while (1)
    {
        for (int i = 0; i < cameraNum; i++)
        {
			zedImage* zedProcessor = zedProcessorVec[i];
			detectInfoSt detectInfo = zedProcessor->detectInfo;
            unsigned char buffer[256] = {0};
			int nRes = dataRead(detectInfo.db, detectInfo.address_r, 1, buffer);
            if (nRes)
			{
				usleep(200*1000);
				continue;
			} 
			printf(".");
            //printf("%s data read :%d db %d address:%d \n", getTime().c_str(), buffer[0], detectInfo.db, detectInfo.address_r);
            unsigned char value = buffer[0] >> detectInfo.offset_r;
            if ((value & 0x01) == 1)
            {
                idleTime = 0;
                if (cameraRunNum < CAM_RUNING_MAX_NUM || i < CAM_RUNING_MAX_NUM)
                {
                    zedImage *zedProcessor;
                    zedProcessor = zedProcessorVec[i];
					if (!zedProcessor->getCameraStatus())
                    {
                        cameraOpen(zedProcessor, i);			//"" for live
                        usleep(500 * 1000);                     //500ms
                    }
                    value = 0x01;
                    value <<= detectInfo.offset_r;
                    buffer[0] &= (~value);
                    dataWrite(detectInfo.db, detectInfo.address_r, 1, buffer);
                    zedWorkStart(i);
                }
            }
            else
            {
                idleTime++;
            }
        }
        if (idleTime > 30) 
        {
            idleTime = 30;
            mainCamKeepLive();
        }
        sleep(1);
    }

    /*    while (1)
    {
        while (!quit)
        {
            for (int i = 0; i < cameraNum; i++)
            {
                cameraProcess(i);
				sl::sleep_ms(500);
            }
        }
    }
*/
    //startZED();

    // Set the display callback
    //glutCloseFunc(close);
    //glutMainLoop();

    return 0;
}

/**
    Launch ZED thread. Using a thread here allows to capture a point cloud and update the GL window concurrently.
 **/
void startZED()
{
    quit = false;
    //zed_callback = std::thread(run);
    pthread_t tidp;
          
     /* 创建线程pthread */
     if ((pthread_create(&tidp, NULL, run, NULL)) == -1)
     {
         printf("create error!\n");
         return ;
     }
}

/**
    This function loops to get image and motion data from the ZED. It is similar to a callback.
    Add your own code here.
 **/
void *run(void *arg)
{

    while (!quit)
    {
        // printf("camera running \n");
        // cv::Mat image, imageR;
        // sl::float4 pointResult;
        // pointResult.x = 0;
        // pointResult.y = 0;
        // pointResult.z = 0;
        // if (zedProcessor.zedImageGet(image, VIEW_LEFT))
        // {
        //     printf("can not read image \n");
        //     continue;
        // }
        // if (!image.empty())
        // {
        //     image.copyTo(imageR);
        // }
        // else
        //     continue;
        // std::vector<cv::Mat> imgPro;
        // printf("image match begin \n");
        // zedProcessor.zed_match(imgPro);
        // zedProcessor.measure(pointResult, imageR);

        // cv::cvtColor(imageR, imageR, CV_BGR2GRAY);
        // printf("image size cols %d rows %d\n", imageR.cols, imageR.rows);

        // cv::imshow("src_image1", imageR);
        // cv::waitKey(5);
        // cv::imwrite("src_image.jpg", imageR);

        // printf("point cloud: x %f  y %f  z %f \n", pointResult.x, pointResult.y, pointResult.z);
        sl::sleep_ms(10);
    }
}

/**
    This function closes the ZED camera, its callback (thread) and the GL viewer
 **/
void close()
{
    quit = true;

    // Stop callback
    zed_callback.join();

    // Exit point cloud viewer
    //viewer.exit();

    // Free buffer and close the ZED
    point_cloud.free(MEM_GPU);
    zed.close();
}

void windowInit()
{
    cv::namedWindow("src_image", 1);
    cv::namedWindow("result1", 1);
    cv::namedWindow("result2", 1);
    cv::namedWindow("result3", 1);
}

void cameraOpen(zedImage *processor, int cameraIndex)
{
    InitParameters initParameters;
    detectInfoSt  &detectInfo = processor->detectInfo;
    initParameters.svo_input_filename.set(detectInfo.videoReadPath.c_str());
    initParameters.coordinate_units = UNIT_MILLIMETER;
    initParameters.depth_mode = DEPTH_MODE_ULTRA; // Use ULTRA depth mode
    initParameters.enable_right_side_measure = true;
    switch (detectInfo.resolution)
    {
    case 0:
        initParameters.camera_resolution = RESOLUTION_HD720;
        break;
    case 1:
        initParameters.camera_resolution = RESOLUTION_HD1080;
        break;
    case 2:
        initParameters.camera_resolution = RESOLUTION_HD2K;
        break;
    default:
        initParameters.camera_resolution = RESOLUTION_HD1080;
        break;
    }
    initParameters.camera_fps = 1;
    initParameters.input.setFromSerialNumber(detectInfo.camera_sn);
    if (!processor->zedOpen(initParameters))
    {
        printf("%s zed open success sn:%d or video path:%s\n", getTime().c_str(),
            detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
        cv::Mat image;
        processor->zedImageGet(image, VIEW_LEFT);
        processor->roiSelect(image, detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_height, 
            detectInfo.roi_width);
        cameraRunNum ++;
       
        image.release();
    }
    else
    {
        printf("zed open failed sn: %d or video path：%s !\n",
            detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
        cameraClose();
        exit(1);
    }
}

void cameraClose()
{
    for (int i = 0; i < zedProcessorVec.size(); i++)
    {
        zedProcessorVec[i]->zedClose();
    }
}