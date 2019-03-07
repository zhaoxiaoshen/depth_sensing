

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

// plc connect 
#include "plcConnect.h"

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

// Point cloud viewer
GLViewer viewer;

float m_fResultX;
float m_fResultY;
float m_fResultZ;

float m_fResultXLive;
float m_fResultYLive;
float m_fResultZLive;

typedef struct detectInfoSt{
    float roi_x;
    float roi_y;
    float roi_width;
    float roi_height; //图像百分比
    int camera_sn;

    std::string templatePath;
    std::string videoReadPath;
    float result[3]; //cloudPoint xyz
    int db;
    int address_r;
    int offset_r;
    int len_r;
    int address_w;
    int offset_w;
    int len_w;
}detectInfoSt;

// Sample functions
void startZED();
void *run(void *arg);
void close();

void windowInit();
void cameraOpen(zedImage& processor, detectInfoSt detectInfo);
void cameraClose();

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
    detectInfo.roi_x = iniFile.GetFloat("ROI_SET", "x");
    detectInfo.roi_y = iniFile.GetFloat("ROI_SET", "y");
    detectInfo.roi_height = iniFile.GetFloat("ROI_SET", "height");
    detectInfo.roi_width = iniFile.GetFloat("ROI_SET", "width");
    detectInfo.templatePath = iniFile.GetStr("TEMPLATE_IMAGE", "path");
    detectInfo.camera_sn = iniFile.GetInt("CAMERA", "sn");
    detectInfo.videoReadPath = iniFile.GetStr("VIDEO","read_path");
    printf("config read roi x:%f y:%f width:%f height:%f \n template path:%s\n camera sn:%d\n video read:%s\n",
        detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_width, detectInfo.roi_height,
        detectInfo.templatePath.c_str(), detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
    detectInfo.db = iniFile.GetInt("DB_READ_SET", "db");
    detectInfo.address_r = iniFile.GetInt("DB_READ_SET", "address");
    detectInfo.address_w = iniFile.GetInt("DB_RESULT_SET", "address");
    detectInfo.offset_r = iniFile.GetInt("DB_READ_SET", "offset");
    printf("db:%d address read:%d offset:%d address write:%d \n",
            detectInfo.db, detectInfo.address_r, detectInfo.offset_r,  detectInfo.address_w);
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

void *cameraProcess(void *argv)
{
    int cameraIndex = *(int *)argv;
    detectInfoSt detectInfo;
    zedImage* zedProcessor;
    detectInfo = detectInfoVec[cameraIndex];
    zedProcessor = zedProcessorVec[cameraIndex];
    printf("camera running index %d\n", cameraIndex);
    cv::Mat image, imageR;
    sl::float4 pointResult;
    pointResult.x = 0;
    pointResult.y = 0;
    pointResult.z = 0;
    if (zedProcessor->zedImageGet(image, VIEW_LEFT))
    {
        printf("can not read image \n");
        return NULL;
    }
    if (!image.empty())
    {
        image.copyTo(imageR);
    }
    else
        return NULL;
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
    //cv::imshow("src_image1", imageR);

    //cv::waitKey(5);
   
    //cv::imwrite(dstNameSave, imageR);

    /*       if (imgPro.size() >= 4)
        {
            imshow("result1", imgPro[1]);
            imshow("result2", imgPro[2]);
            imshow("result3", imgPro[3]);
        }
*/
    printf("%s: point cloud x %f  y %f  z %f \n", getTime().c_str(), pointResult.x, pointResult.y, pointResult.z);
    delete (int *)argv;
    sl::sleep_ms(10);
}

void zedWorkStart(int index)
{
    pthread_t tidp;
    int *cameraIndex = new int(0);
    *cameraIndex = index;
    if ((pthread_create(&tidp, NULL, &cameraProcess, cameraIndex) == -1))
    {
        printf("create zed work error!\n");
        return;
    }
    pthread_detach(tidp);
    printf("camera %d start ....\n", index);
    usleep(5000);
}

int main(int argc, char **argv)
{
    // Start the camera thread

    //==read config.ini
    int cameraNum = 0;
    std::string plcIp = "127.0.0.1";

    CIni iniFile;
    if (iniFile.OpenFile("config/General.ini") != INI_SUCCESS)
    {
        printf("read config file General.ini failed!\n");
        return -1;
    }
    cameraNum = iniFile.GetInt("CAMERA", "num");
    plcIp = iniFile.GetStr("PLC", "ip");
    printf("camera num:%d plc ip:%s \n", cameraNum, plcIp.c_str());


    // printf("plc connect.... \n");
    // plcConnect(plcIp);
    // printf("plc connect end \n");
    // while (1)
    // {
    //     printf("buffer read start \n");
    //     unsigned char buffer[256] = {0};
    //     dataRead(501, 0, 2, buffer);
    //     printf("buffer read :%d \n", buffer[0]);
    //     sleep (1);
    //     dataWrite(501, 4, 10, buffer);
    //     sleep(5);
    // }


    for (int i = 0; i < cameraNum; i++)
    {
        detectInfoSt detectInfo;
        char path[255] = {0};
        snprintf(path, sizeof(path) - 1, "config/Config_%d.ini", i);
        if (iniFile.OpenFile(path) != INI_SUCCESS)
        {
            printf("read config file %s failed!\n", path);
            return -1;
        }
        else
        {
            configInit(detectInfo, iniFile);
            detectInfoVec.push_back(detectInfo);
        }
        // zedImage* zedProcessor = new zedImage();
        // zedProcessor->templateImageLoad(detectInfo.templatePath);
        // cameraOpen(*zedProcessor, detectInfo); //"" for live
        // zedProcessorVec.push_back(zedProcessor);
    }

    // windowInit();
    plcConnect(plcIp);

    quit = false;
    while (1)
    {
        for (int i = 0; i < cameraNum; i++)
        {
            unsigned char buffer[256] = {0};
            dataRead(detectInfoVec[i].db, detectInfoVec[i].address_r, 1, buffer);
            printf("data read :%d db %d address:%d \n", buffer[0],detectInfoVec[i].db,
                detectInfoVec[i].address_r);
            unsigned char value = buffer[0] >> detectInfoVec[i].offset_r;
            if ((value & 0x01) == 1)
            {
                printf("camera %d start\n ", i);
                value = 0x01;
                value <<= detectInfoVec[i].offset_r;
                buffer[0] &= (~value);
                dataWrite(detectInfoVec[0].db, detectInfoVec[0].address_r, 1, buffer);
            }
            //zedWorkStart(i);
        }
        sleep(3);
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
    while (1)
    {
    }
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
    viewer.exit();

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

void cameraOpen(zedImage& processor, detectInfoSt detectInfo)
{
    InitParameters initParameters;
    initParameters.svo_input_filename.set(detectInfo.videoReadPath.c_str());
    initParameters.coordinate_units = UNIT_MILLIMETER;
    initParameters.depth_mode = DEPTH_MODE_ULTRA; // Use ULTRA depth mode
    initParameters.enable_right_side_measure = true;
    //initParameters.camera_resolution = RESOLUTION_HD2K;
    //initParameters.camera_resolution = RESOLUTION_HD1080;
	initParameters.camera_resolution = RESOLUTION_HD720 ;
    initParameters.camera_fps = 1;
    initParameters.input.setFromSerialNumber(detectInfo.camera_sn);
    if (!processor.zedOpen(initParameters))
    {
        printf("zed open success sn:%d or video path:%s\n",
            detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
        cv::Mat image;
        processor.zedImageGet(image, VIEW_LEFT);
        processor.roiSelect(image, detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_height, 
            detectInfo.roi_width);
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