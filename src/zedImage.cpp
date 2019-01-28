#include "zedImage.h"


zedImage::zedImage()
{
}


zedImage::~zedImage()
{
}

void zedImage::init()
{
	imageSize = zed.getResolution();
	width = imageSize.width;
	height = imageSize.height;
	width_sbs = imageSize.width * 2;
	rt_param.sensing_mode = SENSING_MODE_FILL;
	//Mat image(width, height, MAT_TYPE_8U_C4);
	//left_image = image;
	//right_image = image;
	//Mat depth(width, height, MAT_TYPE_32F_C1);
	//depth_image = depth;

}
int  zedImage::zedOpen(InitParameters initParameters)
{
	ERROR_CODE err = zed.open(initParameters);
	if (err != SUCCESS)
	{
		printf("zed open error:%s\n", toString(err).c_str());
		zed.close();
		return 1;
	}
	zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 50, true);

	init();
	nb_frames = zed.getSVONumberOfFrames();
	svo_positoion = 0;

	return 0;
}

int  zedImage::zedClose()
{
	zed.close();
	return 0;
}
int zedImage::zedImageGet(cv::Mat& image, int type)
{
	if (/*!exit_app*/nb_frames==-1||((nb_frames!=-1)&&(svo_positoion<nb_frames-1)))
	{
		if (zed.grab(rt_param) == SUCCESS) {
			zed.retrieveImage(left_image, VIEW_LEFT);
			cv::Mat leftImage = slMat2cvMat(left_image);
			leftImage.copyTo(image);
			leftImage.copyTo(matchInfo.srcImg);
			if (nb_frames!=-1)
				svo_positoion++;
			return 0;
		}
		else
		{
			std::cout << "get picture failed!" << std::endl;
			//cv::waitKey(0);
			return  1;
		}
	}
	else
		return 1;

}
int zedImage::templateImageLoad(std::string imageName)
{
	templateImage = cv::imread(imageName);
	if (templateImage.empty())
	{
		printf("template image read failed!\n");
		return 1;
	}
	else
		return 0;
}

int zedImage::zed_match(std::vector<cv::Mat>& imgPro)
{
	if (templateImage.empty())
	{
		printf("template is null!\n");
		return 1;
	}
	Finder.matchMethod = 5;
	templateImage.copyTo(matchInfo.templateImg);
	Finder.on_Matching(0, &matchInfo, imgPro);
}

int zedImage::measure(sl::float4& point3D,cv::Mat image)
{
	sl::Mat point_cloud;
	if (zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_GPU, width, height) == SUCCESS)
	{
		int xBegin = matchInfo.roiSet.x + matchInfo.roiFind.x;
		int yBeign = matchInfo.roiSet.y + matchInfo.roiFind.y;
		int xEnd = xBegin + matchInfo.roiFind.width;
		int yEnd = yBeign + matchInfo.roiFind.height;
		bool firstPoint = true;
		for (int i = xBegin + 10; i < xEnd - 10; i += 5)
		{
			cv::Point pointDepth = cv::Point(i, (yBeign + yEnd) / 2);
			sl::float4 pointG;
			ERROR_CODE rect = point_cloud.getValue(pointDepth.x, pointDepth.y, &pointG, MEM_GPU);
			if (rect == SUCCESS)
			{
				if (firstPoint)
				{
					firstPoint = false;
					point3D = pointG;
					continue;
				}
				point3D.x = (point3D.x + pointG.x) / 2.0;
				point3D.y = (point3D.y + pointG.y) / 2.0;
				point3D.z = (point3D.z + pointG.z) / 2.0;

				//printf("index:%d %d-%d-%f_%f_%f\n", svo_position, pointDepth.x, pointDepth.y, point3D.x, point3D.y, point3D.z);
				cv::circle(image, pointDepth, 2, cv::Scalar(0, 0, 255), 2, 8, 0);
				//cv::imshow("point3D", leftImg);
				//cv::waitKey(2);
			}
		}
		cv::imwrite("point3D.jpg",image);
	}
	return 0;
}
void zedImage::roiSelect(cv::Mat img)
{
	//Finder.roiSet(img, matchInfo.roiSet);
}

void  zedImage::roiSelect(cv::Mat img, float x, float y, float height, float width)
{
	matchInfo.roiSet.x = img.cols * x;
	matchInfo.roiSet.y = img.rows * y;
	matchInfo.roiSet.width = img.cols * width;
	matchInfo.roiSet.height = img.rows * height;
	printf("roi set x:%d y:%d width:%d height:%d \n",matchInfo.roiSet.x,matchInfo.roiSet.y,
		matchInfo.roiSet.height,matchInfo.roiSet.width);
}

cv::Mat zedImage::slMat2cvMat(sl::Mat &input) {
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
