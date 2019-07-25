#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <math.h>

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

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
	protonect_shutdown = true;
}

int main()
{

	//定义变量
	std::cout << "start!" << std::endl;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline  *pipeline = 0;



	//搜寻并初始化传感器
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "SERIAL: " << serial << std::endl;

	//配置传输格式
#if 1 // sean
	int depthProcessor = Processor_cl;
	if (depthProcessor == Processor_cpu)
	{
		if (!pipeline)
			//! [pipeline]
			pipeline = new libfreenect2::CpuPacketPipeline();
		//! [pipeline]
	}
	else if (depthProcessor == Processor_gl) // if support gl
	{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if (!pipeline)
		{
			pipeline = new libfreenect2::OpenGLPacketPipeline();
		}
#else
		std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
	}
	else if (depthProcessor == Processor_cl) // if support cl
	{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
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
		std::cout << "failure opening device!" << std::endl;
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


	Mat rgbmat, depthmat, rgbd, dst;
	float x, y, z, color;

	pcl::visualization::CloudViewer viewer("Viewer");  //创建一个显示点云的窗口
	


	while (!protonect_shutdown)
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		PointCloud::Ptr cloud(new PointCloud); //使用智能指针，创建一个空点云。这种指针用完会自动释放。
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
		viewer.showCloud(cloud);
		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
		listener.release(frames);
	}

	dev->stop();
	dev->close();

	delete registration;

#endif

	std::cout << "stop!" << std::endl;
	return 0;
}