#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <math.h>
#include <fstream>

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

#include <boost/timer.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vtkAutoInit.h>

#include "shlobj.h"
#include "config.h"
#include "visual_odometry.h"

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
	VTK_MODULE_INIT(vtkRenderingOpenGL);
	VTK_MODULE_INIT(vtkInteractionStyle);
	//定义变量
	cout << "本程序用于实时获取点云，并且获得RGB、深度图" << endl;
	cout << "正在初始化Kinect V2..." << endl;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline  *pipeline = 0;

	//搜寻并初始化传感器
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "没有连接Kinect V2 请确保连接成功后重试..." << std::endl;
		return -1;
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "连接Kinect V2 成功，设备号是: " << serial << std::endl;

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
	else if (depthProcessor == Processor_gl) // 如果支持gl
	{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if (!pipeline)
		{
			pipeline = new libfreenect2::OpenGLPacketPipeline();
		}
#else
		std::cout << "不支持 OpenGL 管线!" << std::endl;
#endif
	}
	else if (depthProcessor == Processor_cl) // 如果支持cl
	{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		std::cout << "不支持 OpenCL 管线!" << std::endl;
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
		std::cout << "设备打开失败！" << std::endl;
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

	std::cout << "设备序列号: " << dev->getSerialNumber() << std::endl;
	std::cout << "设备固件版本: " << dev->getFirmwareVersion() << std::endl;

	//循环接收
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
	cv::viz::Viz3d vis("Visual Odometry");
	Mat rgbmat, depthmat, rgbd, dst;
	float x, y, z, color;

	TCHAR szPath[255];
	SHGetSpecialFolderPath(NULL, szPath, CSIDL_DESKTOP, FALSE);
	string s = szPath;
	string cmd = "md -p " + s + "\\slamprocess\\rgb";
	system(cmd.data());
	string configfilepath = s + "\\slamprocess\\default.yaml";
	
	int i = 0;
	
	while (true)
	{
		ifstream fin(configfilepath);
			if (!fin)
			{
				cout << endl;
				cout << endl;
				cout << endl;
				cout << endl;
				cout << endl;
				cout << "请将配置文件default.yaml放置到桌面slamprocess文件夹下，完成后";
				system("pause");
				continue;
			}
			else {
				break;
			}
	}

	myslam::Config::setParameterFile(configfilepath);
	myslam::Camera::Ptr camera(new myslam::Camera);
	myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
	// visualization
	
	//cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
	//cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
	//cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	//vis.setViewerPose(cam_pose);
	//world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
	//camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
	//vis.showWidget("World", world_coor);
	//vis.showWidget("Camera", camera_coor);
	pcl::visualization::CloudViewer viewer("Viewer");  //创建一个显示点云的窗口
	PointCloud::Ptr cloud(new PointCloud); //使用智能指针，创建一个空点云。这种指针用完会自动释放。
	while (!protonect_shutdown)
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
//
//		//cloud->width = 512 * 424;
//		//cloud->height = 1;
//		//cloud->is_dense = true;
		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::imshow("rgb", rgbmat);
		string ii = to_string(i);
		string pathrgb = s + "\\slamprocess\\rgb\\" + ii + ".jpg";
		cout << "图片存储成功: " << pathrgb << endl;
		cv::imwrite(pathrgb, rgbmat);

		cout << "****** loop " << i << " ******" << endl;
		Mat colormat = rgbmat;
//		//Mat depth = cv::imread(depth_files[i], -1);
//		//if (color.data == nullptr || depth.data == nullptr)
//		if (colormat.data == NULL)
//			break;
		myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
		pFrame->camera_ = camera;
		pFrame->color_ = colormat;
		pFrame->time_stamp_ = i;

		boost::timer timer;
		vo->addFrame(pFrame);
		//cout << "VO costs time: " << timer.elapsed() << endl;

		//if (vo->state_ == myslam::VisualOdometry::LOST)
			//b reak;
		//SE3 Twc = pFrame->T_c_w_.inverse();

		// show the map and the camera pose
		cv::Affine3d M(
			cv::Affine3d::Mat3(
				pFrame->T_c_w_.inverse().rotation_matrix() (0, 0), pFrame->T_c_w_.inverse().rotation_matrix() (0, 1), pFrame->T_c_w_.inverse().rotation_matrix() (0, 2),
				pFrame->T_c_w_.inverse().rotation_matrix() (1, 0), pFrame->T_c_w_.inverse().rotation_matrix() (1, 1), pFrame->T_c_w_.inverse().rotation_matrix() (1, 2),
				pFrame->T_c_w_.inverse().rotation_matrix() (2, 0), pFrame->T_c_w_.inverse().rotation_matrix() (2, 1), pFrame->T_c_w_.inverse().rotation_matrix() (2, 2)
			),
			cv::Affine3d::Vec3(
				pFrame->T_c_w_.inverse().translation() (0, 0), pFrame->T_c_w_.inverse().translation() (1, 0), pFrame->T_c_w_.inverse().translation() (2, 0)
			)
		);

		Mat img_show = colormat.clone();
		for (auto& pt : vo->map_->map_points_)
		{
			myslam::MapPoint::Ptr p = pt.second;
			Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->T_c_w_);
			cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
		}

		cv::imshow("image", img_show);
		cv::waitKey(1);
		vis.setWidgetPose("Camera", M);
		vis.spinOnce(1, false);
		cout << endl;
//
//		for (int m = 0; m < 512; m++)
//		{
//			for (int n = 0; n < 424; n++)
//			{
//				PointT p;
//				registration->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
//				const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
//				uint8_t b = c[0];
//				uint8_t g = c[1];
//				uint8_t r = c[2];
//				if (z < 1.2 && y < 0.2)  //暂时先通过限定xyz来除去不需要的点，点云分割还在学习中。。。
//				{
//					p.z = -z;
//					p.x = -x;
//					p.y = -y;
//					p.b = b;
//					p.g = g;
//					p.r = r;
//				}
//				cloud->points.push_back(p);
//			}
//		}
//		viewer.showCloud(cloud);
//		int key = cv::waitKey(1);
//		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
//		listener.release(frames);
//		Sleep(500);
//		i++;
	}
	dev->stop();
	dev->close();
	
	delete registration;

#endif

	std::cout << "程序停止!" << std::endl;
}