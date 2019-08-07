#pragma once
#include "slamBases.h"

/**
 * Get the value of a specified key from a std::map.
 * @param m the map
 * @param key the key
 * @param defaultValue the default value used if the key is not found
 * @return the value
 */
template<class K, class V>
inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
	V v = defaultValue;
	typename std::map<K, V>::const_iterator i = m.find(key);
	if (i != m.end())
	{
		v = i->second;
	}
	return v;
}

//全局变量声明
int deviceId_; //设备id
libfreenect2::Freenect2* freenect2_; 
libfreenect2::SyncMultiFrameListener* listener_; //监听器
libfreenect2::Freenect2Device* dev_; //设备
libfreenect2::Registration * reg_; //注册器

//功能：初始化Kinect V2
//参数：无
//返回值：初始化成功返回true，失败返回false
bool init();
//获得图像信息
//参数：拍摄帧序号
//返回值：成功返回true，失败返回false
bool captureImage(string id);
//获得图像信息
//参数：输出的彩色图矩阵，输出的深度图矩阵
//返回值：成功返回true，失败返回false
bool captureImage(cv::Mat &colorMat,cv::Mat &depthMat);
//获得图像信息
//参数：拍摄帧序号，输出的彩色图矩阵，输出的深度图矩阵
//返回值：成功返回true，失败返回false
bool captureImage(string id,cv::Mat &colorMat, cv::Mat &depthMat);
//获得图像信息和点云
//参数：拍摄帧序号
//返回值：合成的点云
PointCloud::Ptr captureImagecloud(string id);
//获得图像信息和点云
//参数：输出的彩色图矩阵，输出的深度图矩阵
//返回值：合成的点云
PointCloud::Ptr captureImagecloud(cv::Mat &colorMat, cv::Mat &depthMat);
//获得图像信息和点云
//参数：拍摄帧序号，输出的彩色图矩阵，输出的深度图矩阵
//返回值：合成的点云
PointCloud::Ptr captureImagecloud(string id, cv::Mat &colorMat, cv::Mat &depthMat);

//获得相机参数
//参数：无
//返回值：相机参数结构体
inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
	libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
	CAMERA_INTRINSIC_PARAMETERS camera;
	camera.fx = params.fx;
	camera.fy = params.fy;
	camera.cx = params.cx;
	camera.cy = params.cy;
	camera.scale = 1000.0;
	return camera;
}

//主函数示例
//int main()
//{
//	freenect2_ = new libfreenect2::Freenect2();
//	listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
//
//	init();

//	delete freenect2_;
//	delete listener_;
//	return 0;
//}