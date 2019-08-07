#pragma once

// ����ͷ�ļ� 
// C++��׼��
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <math.h>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include "shlobj.h"

// libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/config.h>

// g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

// ���Ͷ���
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ����ڲνṹ
struct CAMERA_INTRINSIC_PARAMETERS
{
	double cx, cy, fx, fy, scale;
};

// ֡�ṹ
struct FRAME
{
	int frameID;
	cv::Mat rgb, depth; //��֡��Ӧ�Ĳ�ɫͼ�����ͼ
	cv::Mat desp;       //����������
	vector<cv::KeyPoint> kp; //�ؼ���
};

// PnP ���
struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};

// ������ȡ��
class ParameterReader
{
public:
	ParameterReader(string filename = "./parameters.txt")
	{
		ifstream fin(filename.c_str());
		if (!fin)
		{
			cerr << "�����ļ��Ҳ������뽫�����ļ����ڳ���ͳ��Ŀ¼�£�������Ϊparameters.txt" << endl;
			return;
		}
		while (!fin.eof())
		{
			string str;
			getline(fin, str);
			if (str[0] == '#')
			{
				// �ԡ�������ͷ����ע��
				continue;
			}

			int pos = str.find("=");
			if (pos == -1)
				continue;
			string key = str.substr(0, pos);
			string value = str.substr(pos + 1, str.length());
			data[key] = value;

			if (!fin.good())
				break;
		}
	}

	string getData(string key)
	{
		map<string, string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr << "�����ֶ� " << key << " �޷��ҵ�!" << endl;
			return string("NOT_FOUND");
		}
		return iter->second;
	}

public:
	map<string, string> data;
};

// �����ӿ�
// image2PonitCloud ��rgbͼת��Ϊ����
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d ���������ͼ������ת��Ϊ�ռ�����
// input: 3ά��Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);

// computeKeyPointsAndDesp ͬʱ��ȡ�ؼ���������������
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor);

// estimateMotion ��������֮֡����˶�
// ���룺֡1��֡2, ����ڲ�
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);

// joinPointCloud 
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);
//PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);

// ����index����ȡһ֡����
FRAME readFrame(int index, ParameterReader& pd);

// ����һ���˶��Ĵ�С
double normofTransform(cv::Mat rvec, cv::Mat tvec);

//���ؽ�����
PointCloud::Ptr voxelDownsample(PointCloud::Ptr original);