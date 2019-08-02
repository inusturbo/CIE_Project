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
	//�������
	cout << "����������ʵʱ��ȡ���ƣ����һ��RGB�����ͼ" << endl;
	cout << "���ڳ�ʼ��Kinect V2..." << endl;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline  *pipeline = 0;


	//��Ѱ����ʼ��������
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "û������Kinect V2 ��ȷ�����ӳɹ�������..." << std::endl;
		return -1;
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "����Kinect V2 �ɹ����豸����: " << serial << std::endl;

	//���ô����ʽ
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
		std::cout << "��֧�� OpenGL ����!" << std::endl;
#endif
	}
	else if (depthProcessor == Processor_cl) // if support cl
	{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		std::cout << "��֧�� OpenCL ����!" << std::endl;
#endif
	}



	//�����豸
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
		std::cout << "�豸��ʧ�ܣ�" << std::endl;
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

	//�������ݴ���
	dev->start();

	std::cout << "�豸���к�: " << dev->getSerialNumber() << std::endl;
	std::cout << "�豸�̼��汾: " << dev->getFirmwareVersion() << std::endl;

	//ѭ������
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
	myslam::Config::setParameterFile(configfilepath);
	myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
	int i = 0;
	myslam::Camera::Ptr camera(new myslam::Camera);

	// visualization
	
	cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
	cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
	cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	vis.setViewerPose(cam_pose);
	world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
	camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
	vis.showWidget("World", world_coor);
	vis.showWidget("Camera", camera_coor);
	pcl::visualization::CloudViewer viewer("Viewer");  //����һ����ʾ���ƵĴ���
	PointCloud::Ptr cloud(new PointCloud); //ʹ������ָ�룬����һ���յ��ơ�����ָ��������Զ��ͷš�
	while (!protonect_shutdown)
	{
		ifstream fin(configfilepath);
		if (!fin)
		{
			cout << endl;
			cout << endl;
			cout << endl;
			cout << endl;
			cout << endl;
			cout << "�뽫�����ļ�default.yaml���õ�����slamprocess�ļ����£���ɺ�";
			system("pause");
			continue;
		}
		
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		//cloud->width = 512 * 424;
		//cloud->height = 1;
		//cloud->is_dense = true;
		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::imshow("rgb", rgbmat);
		string ii = to_string(i);
		string pathrgb = s + "\\slamprocess\\rgb\\" + ii + ".jpg";
		cout << "ͼƬ�洢�ɹ�: " << pathrgb << endl;
		cv::imwrite(pathrgb, rgbmat);

		cout << "****** loop " << i << " ******" << endl;
		Mat colormat = rgbmat;
		//Mat depth = cv::imread(depth_files[i], -1);
		//if (color.data == nullptr || depth.data == nullptr)
		if (colormat.data == NULL)
			break;
		myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
		pFrame->camera_ = camera;
		pFrame->color_ = colormat;
		//pFrame->depth_ = depth;
		pFrame->time_stamp_ = i;

		boost::timer timer;
		vo->addFrame(pFrame);
		cout << "VO costs time: " << timer.elapsed() << endl;

		if (vo->state_ == myslam::VisualOdometry::LOST)
			//break;
		SE3 Twc = pFrame->T_c_w_.inverse();

			// show the map and the camera pose
		cv::Affine3d M(
			cv::Affine3d::Mat3(
				Twc.rotation_matrix() (0, 0), Twc.rotation_matrix() (0, 1), Twc.rotation_matrix() (0, 2),
				Twc.rotation_matrix() (1, 0), Twc.rotation_matrix() (1, 1), Twc.rotation_matrix() (1, 2),
				Twc.rotation_matrix() (2, 0), Twc.rotation_matrix() (2, 1), Twc.rotation_matrix() (2, 2)
			),
			cv::Affine3d::Vec3(
				Twc.translation() (0, 0), Twc.translation() (1, 0), Twc.translation() (2, 0)
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
				if (z < 1.2 && y < 0.2)  //��ʱ��ͨ���޶�xyz����ȥ����Ҫ�ĵ㣬���Ʒָ��ѧϰ�С�����
				{
					p.z = -z;
					p.x = -x;
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
		Sleep(500);
		i++;
	}

	dev->stop();
	dev->close();

	delete registration;

#endif

	std::cout << "����ֹͣ!" << std::endl;










	

	//string dataset_dir = myslam::Config::get<string>("dataset_dir");
	//cout << "dataset: " << dataset_dir << endl;
	//ifstream fin(dataset_dir + "/associate.txt");
	//if (!fin)
	//{
	//	cout << "please generate the associate file called associate.txt!" << endl;
	//	return 1;
	//}

	//vector<string> rgb_files, depth_files;
	//vector<double> rgb_times, depth_times;
	//while (!fin.eof())
	//{
	//	string rgb_time, rgb_file, depth_time, depth_file;
	//	fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
	//	rgb_times.push_back(atof(rgb_time.c_str()));
	//	depth_times.push_back(atof(depth_time.c_str()));
	//	rgb_files.push_back(dataset_dir + "/" + rgb_file);
	//	depth_files.push_back(dataset_dir + "/" + depth_file);

	//	if (fin.good() == false)
	//		break;
	//}

	






	

	//cout << "read total " << rgb_files.size() << " entries" << endl;
	//for (int i = 0; i < rgb_files.size(); i++)
	//{
	//	cout << "****** loop " << i << " ******" << endl;
	//	Mat color = cv::imread(rgb_files[i]);
	//	Mat depth = cv::imread(depth_files[i], -1);
	//	if (color.data == nullptr || depth.data == nullptr)
	//		break;
	//	myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
	//	pFrame->camera_ = camera;
	//	pFrame->color_ = color;
	//	pFrame->depth_ = depth;
	//	pFrame->time_stamp_ = rgb_times[i];

	//	boost::timer timer;
	//	vo->addFrame(pFrame);
	//	cout << "VO costs time: " << timer.elapsed() << endl;

	//	if (vo->state_ == myslam::VisualOdometry::LOST)
	//		break;
	//	SE3 Twc = pFrame->T_c_w_.inverse();

	//	// show the map and the camera pose
	//	cv::Affine3d M(
	//		cv::Affine3d::Mat3(
	//			Twc.rotation_matrix() (0, 0), Twc.rotation_matrix() (0, 1), Twc.rotation_matrix() (0, 2),
	//			Twc.rotation_matrix() (1, 0), Twc.rotation_matrix() (1, 1), Twc.rotation_matrix() (1, 2),
	//			Twc.rotation_matrix() (2, 0), Twc.rotation_matrix() (2, 1), Twc.rotation_matrix() (2, 2)
	//		),
	//		cv::Affine3d::Vec3(
	//			Twc.translation() (0, 0), Twc.translation() (1, 0), Twc.translation() (2, 0)
	//		)
	//	);

	//	Mat img_show = color.clone();
	//	for (auto& pt : vo->map_->map_points_)
	//	{
	//		myslam::MapPoint::Ptr p = pt.second;
	//		Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->T_c_w_);
	//		cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
	//	}

	//	cv::imshow("image", img_show);
	//	cv::waitKey(1);
	//	vis.setWidgetPose("Camera", M);
	//	vis.spinOnce(1, false);
	//	cout << endl;
	//}

	//return 0;
}
