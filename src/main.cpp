///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*************************************************************************
 ** This sample demonstrates how to capture images and 3D point cloud   **
 ** with the ZED SDK and display the result in an OpenGL window. 		    **
 *************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
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
}detectInfoSt;

// Sample functions
void startZED();
void *run(void *arg);
void close();

void windowInit();
void cameraOpen(zedImage& processor, detectInfoSt detectInfo);

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
    printf("config read roi x:%f y:%f width:%f height:%f \n template path:%s\n camera sn:%d\n video read:%s",
        detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_width, detectInfo.roi_height,
        detectInfo.templatePath.c_str(), detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
}
 
std::string getTime(void)
{
    char time_str[100]={0};
    time_t now;
    struct tm *tm_now;
 
    time(&now);
    tm_now = gmtime(&now);
    snprintf(time_str,sizeof(time_str)-1,"%04d_%02d_%02d %02d:%02d:%02d \n",
        tm_now->tm_year, tm_now->tm_mon, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
    
    return time_str;
}


zedImage zedProcessor;
detectInfoSt detectInfo;

int main(int argc, char **argv)
{
    // Start the camera thread

    //==read config.ini
    int cameraNum = 0;
    std::string plcIp = "127.0.0.1";

    CIni iniFile;
    if (iniFile.OpenFile("General.ini") != INI_SUCCESS)
    {
        printf("read config file General.ini failed!\n");
        return -1;
    }
    cameraNum = iniFile.GetInt("CAMERA", "num");
    plcIp = iniFile.GetStr("PLC", "ip");
    printf("camera num:%d plc ip:%s \n", cameraNum, plcIp.c_str());

    for (int i = 0; i < cameraNum; i++)
    {
        char path[255] = {0};
        snprintf(path, sizeof(path) - 1, "Config_%d.ini", i);
        if (iniFile.OpenFile(path) != INI_SUCCESS)
        {
            printf("read config file %s failed!\n", path);
            return -1;
        }
        else
        {
            configInit(detectInfo, iniFile);
        }
    }
    //return 0;

    // windowInit();
    plcConnect(plcIp);

    zedProcessor.templateImageLoad(detectInfo.templatePath);
    cameraOpen(zedProcessor,detectInfo); //"" for live
    
    quit = false;
    while (1)
    {
        while (!quit)
        {
            printf("camera running \n");
            cv::Mat image, imageR;
            sl::float4 pointResult;
            pointResult.x = 0;
            pointResult.y = 0;
            pointResult.z = 0;
            if (zedProcessor.zedImageGet(image, VIEW_LEFT))
            {
                printf("can not read image \n");
                continue;
            }
            if (!image.empty())
            {
                image.copyTo(imageR);
            }
            else
                continue;
            std::vector<cv::Mat> imgPro;

            printf("image match begin \n");
            zedProcessor.zed_match(imgPro);
            zedProcessor.measure(pointResult, imageR);

            cv::cvtColor(imageR, imageR, CV_BGR2GRAY);
            printf("image size cols %d rows %d\n", imageR.cols, imageR.rows);

            //cv::imshow("src_image1", imageR);

            //cv::waitKey(5);
            cv::imwrite("src_image.jpg", imageR);

            /*       if (imgPro.size() >= 4)
        {
            imshow("result1", imgPro[1]);
            imshow("result2", imgPro[2]);
            imshow("result3", imgPro[3]);
        }
*/
            printf("%s point cloud: x %f  y %f  z %f \n",getTime().c_str(), pointResult.x, pointResult.y, pointResult.z);
            sl::sleep_ms(10);
        }
    }

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
        printf("camera running \n");
        cv::Mat image, imageR;
        sl::float4 pointResult;
        pointResult.x = 0;
        pointResult.y = 0;
        pointResult.z = 0;
        if (zedProcessor.zedImageGet(image, VIEW_LEFT))
        {
            printf("can not read image \n");
            continue;
        }
        if (!image.empty())
        {
            image.copyTo(imageR);
        }
        else
            continue;
        std::vector<cv::Mat> imgPro;
        printf("image match begin \n");
        zedProcessor.zed_match(imgPro);
        zedProcessor.measure(pointResult, imageR);

        cv::cvtColor(imageR, imageR, CV_BGR2GRAY);
        printf("image size cols %d rows %d\n", imageR.cols, imageR.rows);

        cv::imshow("src_image1", imageR);
        cv::waitKey(5);
        cv::imwrite("src_image.jpg", imageR);

        /*       if (imgPro.size() >= 4)
        {
            imshow("result1", imgPro[1]);
            imshow("result2", imgPro[2]);
            imshow("result3", imgPro[3]);
        }
*/
        printf("point cloud: x %f  y %f  z %f \n", pointResult.x, pointResult.y, pointResult.z);
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
    initParameters.camera_resolution = RESOLUTION_HD2K;
    initParameters.input.setFromSerialNumber(detectInfo.camera_sn);
    if (!processor.zedOpen(initParameters))
    {
        cv::Mat image;
        processor.zedImageGet(image, VIEW_LEFT);
        processor.roiSelect(image, detectInfo.roi_x, detectInfo.roi_y, detectInfo.roi_height, 
            detectInfo.roi_width);
    }
    else
    {
        printf("zed open failed sn: %d or video path：%s !\n",
            detectInfo.camera_sn, detectInfo.videoReadPath.c_str());
    }
}
