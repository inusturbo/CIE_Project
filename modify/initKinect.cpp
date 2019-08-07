#include "initKinect.h"

bool init()
{
	libfreenect2::PacketPipeline* pipeline;
	pipeline = new libfreenect2::CpuPacketPipeline();

	std::cout << ">>> ���ڴ�Kinect V2 ..." << std::endl;
	dev_ = freenect2_->openDefaultDevice(pipeline);
	pipeline = 0;

	if (dev_)
	{
		std::cout << ">>> ��Kinect V2�ɹ��� " << std::endl;
		libfreenect2::Freenect2Device::Config config;
		config.EnableBilateralFilter = true;
		config.EnableEdgeAwareFilter = true;
		config.MinDepth = 0.3f;
		config.MaxDepth = 12.0f;
		dev_->setConfiguration(config);

		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);

		dev_->start();

		std::cout << ">>> CameraFreenect2: �豸���к�: " << dev_->getSerialNumber() << std::endl;
		std::cout << ">>> CameraFreenect2: �豸Ӳ���汾: " << dev_->getFirmwareVersion() << std::endl;

		//default registration params
		libfreenect2::Freenect2Device::IrCameraParams depthParams = dev_->getIrCameraParams();
		libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev_->getColorCameraParams();
		reg_ = new libfreenect2::Registration(depthParams, colorParams);

	}

	return true;
}
//�ú������ڱ����ɫ�����ͼ
bool captureImage(string id)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
			return false;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4),
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			cv::Mat rgb, depth;

			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgb);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);
				cv::flip(rgb, rgb, 1);//ͼ��ת
				cv::flip(depth, depth, 1);

				string pathrgb = ".//rgb//" + id + ".jpg";
				string pathdepth = ".//depth//" + id + ".png";
				cv::imwrite(pathrgb, rgb);
				cout << ">>> �����ɫͼ��ɹ���·����" << pathrgb << endl;
				cv::imwrite(pathdepth, depth);
				cout << ">>> ���汣�����ͼ��ɹ���·����" << pathdepth << endl;
			}
			listener_->release(frames);
			return true;
		}
	}
}
bool captureImage(cv::Mat &colorMat, cv::Mat &depthMat)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
			return false;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4),
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(colorMat);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depthMat, CV_16U, 1);
				cv::flip(colorMat, colorMat, 1);//ͼ��ת
				cv::flip(depthMat, depthMat, 1);
			}
			listener_->release(frames);
			return true;
		}
	}
}
bool captureImage(string id, cv::Mat &colorMat, cv::Mat &depthMat)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
			return false;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4),
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(colorMat);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depthMat, CV_16U, 1);
				cv::flip(colorMat, colorMat, 1);//ͼ��ת
				cv::flip(depthMat, depthMat, 1);

				string pathrgb = ".//rgb//" + id + ".jpg";
				string pathdepth = ".//depth//" + id + ".png";
				cv::imwrite(pathrgb, colorMat);
				cout << ">>> �����ɫͼ��ɹ���·����" << pathrgb << endl;
				cv::imwrite(pathdepth, depthMat);
				cout << ">>> ���汣�����ͼ��ɹ���·����" << pathdepth << endl;
			}
			listener_->release(frames);
			return true;
		}
	}
}






//�ú������ڱ����ɫ�����ͼ�����ҷ������ɵĵ���
PointCloud::Ptr captureImagecloud(string id)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4), 
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			cv::Mat rgb, depth;
			
			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgb);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);
				cv::flip(rgb, rgb, 1);//ͼ��ת
				cv::flip(depth, depth, 1);

				string pathrgb = ".//rgb//" + id + ".jpg";
				string pathdepth = ".//depth//" + id + ".png";
				cv::imwrite(pathrgb,rgb);
				cout << ">>> �����ɫͼ��ɹ���·����" << pathrgb << endl;
				cv::imwrite(pathdepth, depth);
				cout << ">>> ���汣�����ͼ��ɹ���·����" << pathdepth << endl;
			}
			PointCloud::Ptr cloud(new PointCloud);
			float x, y, z, color;
			for (int m = 0; m < 512; m++)
			{
				for (int n = 0; n < 424; n++)
				{

					PointT p;
					reg_->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
					const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
					uint8_t b = c[0];
					uint8_t g = c[1];
					uint8_t r = c[2];
					//if (x > 0 && y > 0 && z > 0 && z < 1.2 && y < 0.2)  //��ʱ��ͨ���޶�xyz����ȥ����Ҫ�ĵ㣬���Ʒָ��ѧϰ�С�����
					//{
						p.z = -z;
						p.x = -x;
						p.y = -y;
						p.b = b;
						p.g = g;
						p.r = r;
					//}
					cloud->points.push_back(p);
				}
			}
			cloud->height = 1;
			cloud->width = cloud->points.size();
			cloud->is_dense = false;
			cout << ">>> ���� " + id + "�����ɣ�" << endl;
			listener_->release(frames);
			return cloud;
			
		}
	}
}


PointCloud::Ptr captureImagecloud(cv::Mat &colorMat, cv::Mat &depthMat)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4),
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			cv::Mat rgb, depth;

			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(colorMat);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depthMat, CV_16U, 1);
				cv::flip(colorMat, colorMat, 1);//ͼ��ת
				cv::flip(depthMat, depthMat, 1);
			}
			PointCloud::Ptr cloud(new PointCloud);
			float x, y, z, color;
			for (int m = 0; m < 512; m++)
			{
				for (int n = 0; n < 424; n++)
				{

					PointT p;
					reg_->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
					const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
					uint8_t b = c[0];
					uint8_t g = c[1];
					uint8_t r = c[2];
					//if (x > 0 && y > 0 && z > 0 && z < 1.2 && y < 0.2)  //��ʱ��ͨ���޶�xyz����ȥ����Ҫ�ĵ㣬���Ʒָ��ѧϰ�С�����
					//{
					p.z = -z;
					p.x = -x;
					p.y = -y;
					p.b = b;
					p.g = g;
					p.r = r;
					//}
					cloud->points.push_back(p);
				}
			}
			cloud->height = 1;
			cloud->width = cloud->points.size();
			cloud->is_dense = false;
			cout << ">>> ���������ɣ�" << endl;
			listener_->release(frames);
			return cloud;

		}
	}
}
PointCloud::Ptr captureImagecloud(string id, cv::Mat &colorMat, cv::Mat &depthMat)
{
	if (dev_ && listener_)
	{
		libfreenect2::FrameMap frames;

		if (!listener_->waitForNewFrame(frames, 1000))
		{
			std::cout << "*** CameraFreenect2: ���֡����ʧ��!" << std::endl;
		}
		else
		{
			libfreenect2::Frame* rgbFrame = 0;
			libfreenect2::Frame* depthFrame = 0;
			libfreenect2::Frame undistorted(512, 424, 4),
				registered(512, 424, 4),
				color_depth_map(1920, 1080 + 2, 4);

			rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
			depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
			reg_->apply(rgbFrame, depthFrame, &undistorted, &registered, true, &color_depth_map);

			cv::Mat rgb, depth;

			if (rgbFrame && depthFrame)
			{
				//cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(rgb);
				cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(colorMat);
				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depthMat, CV_16U, 1);
				cv::flip(colorMat, colorMat, 1);//ͼ��ת
				cv::flip(depthMat, depthMat, 1);

				libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
				string pathrgb = ".//rgb//" + id + ".jpg";
				string pathdepth = ".//depth//" + id + ".png";
				cv::imwrite(pathrgb, colorMat);
				cout << ">>> �����ɫͼ��ɹ���·����" << pathrgb << endl;
				cv::imwrite(pathdepth, depthMat);
				cout << ">>> ���汣�����ͼ��ɹ���·����" << pathdepth << endl;
			}
			PointCloud::Ptr cloud(new PointCloud);
			float x, y, z, color;
			for (int m = 0; m < 512; m++)
			{
				for (int n = 0; n < 424; n++)
				{

					PointT p;
					reg_->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
					const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
					uint8_t b = c[0];
					uint8_t g = c[1];
					uint8_t r = c[2];
					//if (x > 0 && y > 0 && z > 0 && z < 1.2 && y < 0.2)  //��ʱ��ͨ���޶�xyz����ȥ����Ҫ�ĵ㣬���Ʒָ��ѧϰ�С�����
					//{
					p.z = -z;
					p.x = -x;
					p.y = -y;
					p.b = b;
					p.g = g;
					p.r = r;
					//}
					cloud->points.push_back(p);
				}
			}
			cloud->height = 1;
			cloud->width = cloud->points.size();
			cloud->is_dense = false;
			cout << ">>> ���� " + id + "�����ɣ�" << endl;
			listener_->release(frames);
			return cloud;

		}
	}
}