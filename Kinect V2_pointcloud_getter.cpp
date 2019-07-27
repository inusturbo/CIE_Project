#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "Windows.h"
#include "shlobj.h"
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

bool protonect_shutdown = false; // 是否将运行的程序关闭

void sigint_handler(int s)
{
	protonect_shutdown = true;
}
int main()
{
	//传感器初始化

	//定义变量
	cout << "本程序用于实时获取点云，并且获得RGB、深度图" << endl;
	cout << "正在初始化Kinect V2..." << endl;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	//搜寻并初始化传感器
	if (freenect2.enumerateDevices() == 0)
	{
		cout << "没有连接Kinect V2 请确保连接成功后重试..." << endl;
		return -1;
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	cout << "连接Kinect V2 成功，设备号是：" << serial << endl;

#if 1
	int depthProcessor = Processor_cl;
	if (depthProcessor == Processor_cpu)
	{
		if (!pipeline)
			pipeline = new libfreenect2::CpuPacketPipeline();
	}
	else if (depthProcessor == Processor_gl)//如果支持gl
	{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if (!pipeline)
		{
			pipeline = new libfreenect2::OpenGLPacketPipeline();
		}
#else
		cout << "OpenGL 管线不被支持！" << endl;
#endif // LIBFREENECT2_WITH_OPENGL_SUPPORT
	}
	else if (depthProcessor == Processor_cl)//如果支持cl
	{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		cout << "OpenCL 管线不被支持！" << endl;
#endif // LIBFREENECT2_WITH_OPENCL_SUPPORT
	}

	//启动设备
	if (pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}
	if (dev == 0)
	{
		cout << "设备打开失败！" << endl;
		return -1;
	}
	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;
	libfreenect2::SyncMultiFrameListener listener(
		libfreenect2::Frame::Color |
		libfreenect2::Frame::Depth |
		libfreenect2::Frame::Ir);
	libfreenect2::FrameMap frames;
	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	//启动数据传输
	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	//循环接收
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
	int i = 0;
	while (true)
	{
		
		TCHAR szPath[255];
		SHGetSpecialFolderPath(NULL, szPath, CSIDL_DESKTOP, FALSE);
		Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2, dst;
		float x, y, z, color;

		//cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("ir", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("undistorted", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("registered", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("depth2RGB", WND_PROP_ASPECT_RATIO);

		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
		PointCloud::Ptr cloud(new PointCloud); //使用智能指针，创建一个空点云。这种指针用完会自动释放。
		cloud->width = 512 * 424;
		cloud->height = 1;
		cloud->is_dense = true;

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		//cv::imshow("rgb", rgbmat);
		//cv::imshow("ir", irmat / 4500.0f);
		//cv::imshow("depth", depthmat / 4500.0f);
		//cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
		//cv::imshow("registered", rgbd);
		//cv::imshow("depth2RGB", rgbd2 / 4500.0f);

		//ir：红外图
		//depth：深度图
		//rgb：彩色图
		//depth2RGB：深度图转换为RGB
		//registered：彩色图注入深度图
		//undistorted：深度图（无失真）
		string ii = to_string(i);
		string s = szPath;
		string pathrgb = s + "\\" + ii + "\\rgb.jpg";
		string pathir = s + "\\" + ii + "\\ir.jpg";
		string pathdepth = s + "\\" + ii + "\\depth.jpg";
		string pathundistorted = s + "\\" + ii + "\\undistorted.jpg";
		string pathregistered = s + "\\" + ii + "\\registered.jpg";
		string pathdepth2RGB = s + "\\" + ii + "\\depth2RGB.jpg";
		string pathpointcloud = s+"\\"+ii+"\\PointCloud.pcd";
		cv::imwrite(pathrgb, rgbmat);
		cv::imwrite(pathir, irmat);
		cv::imwrite(pathdepth, depthmat);
		cv::imwrite(pathundistorted, depthmatUndistorted);
		cv::imwrite(pathregistered, rgbd);
		cv::imwrite(pathdepth2RGB, rgbd2);

		for (int m = 0; m < 512; m++)
		{
			for (int n = 0; n < 424; n++)
			{
				PointT p;
				registration->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
				const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
				uint8_t b = c[0];
				uint8_t g = c[1];
				uint8_t r = c[2];
				if (z < 1.2 && y < 0.2)  //暂时先通过限定xyz来除去不需要的点，点云分割还在学习中。。。
				{
					p.z = -z;
					p.x = x;
					p.y = -y;
					p.b = b;
					p.g = g;
					p.r = r;
				}
				cloud->points.push_back(p);
			}
		}
		cloud->points.resize(cloud->width * cloud->height);
		pcl::visualization::CloudViewer viewer("Viewer");
		viewer.showCloud(cloud);
		
		
		string pathpointcloud = s+"//"+ii+"//PointCloud.pcd";

		pcl::io::savePCDFile(pathpointcloud, *cloud);
		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
		listener.release(frames);
		system("pause");
		i++;

		

	}
	

	//关闭设备
	dev->stop();
	dev->close();

	delete registration;
#endif

}
