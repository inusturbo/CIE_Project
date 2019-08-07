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

//ȫ�ֱ�������
int deviceId_; //�豸id
libfreenect2::Freenect2* freenect2_; 
libfreenect2::SyncMultiFrameListener* listener_; //������
libfreenect2::Freenect2Device* dev_; //�豸
libfreenect2::Registration * reg_; //ע����

//���ܣ���ʼ��Kinect V2
//��������
//����ֵ����ʼ���ɹ�����true��ʧ�ܷ���false
bool init();
//���ͼ����Ϣ
//����������֡���
//����ֵ���ɹ�����true��ʧ�ܷ���false
bool captureImage(string id);
//���ͼ����Ϣ
//����������Ĳ�ɫͼ������������ͼ����
//����ֵ���ɹ�����true��ʧ�ܷ���false
bool captureImage(cv::Mat &colorMat,cv::Mat &depthMat);
//���ͼ����Ϣ
//����������֡��ţ�����Ĳ�ɫͼ������������ͼ����
//����ֵ���ɹ�����true��ʧ�ܷ���false
bool captureImage(string id,cv::Mat &colorMat, cv::Mat &depthMat);
//���ͼ����Ϣ�͵���
//����������֡���
//����ֵ���ϳɵĵ���
PointCloud::Ptr captureImagecloud(string id);
//���ͼ����Ϣ�͵���
//����������Ĳ�ɫͼ������������ͼ����
//����ֵ���ϳɵĵ���
PointCloud::Ptr captureImagecloud(cv::Mat &colorMat, cv::Mat &depthMat);
//���ͼ����Ϣ�͵���
//����������֡��ţ�����Ĳ�ɫͼ������������ͼ����
//����ֵ���ϳɵĵ���
PointCloud::Ptr captureImagecloud(string id, cv::Mat &colorMat, cv::Mat &depthMat);

//����������
//��������
//����ֵ����������ṹ��
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

//������ʾ��
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