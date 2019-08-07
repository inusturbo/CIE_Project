#include "initKinect.h"

// image2PonitCloud ��rgbͼת��Ϊ����
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// ��ȡ���ͼ��(m,n)����ֵ
			ushort d = depth.ptr<ushort>(m)[n];
			// d ����û��ֵ������ˣ������˵�
			if (d == 0)
				continue;
			// d ����ֵ�������������һ����
			PointT p;

			// ���������Ŀռ�����
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;

			// ��rgbͼ���л�ȡ������ɫ
			// rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// ��p���뵽������
			cloud->points.push_back(p);
		}
	// ���ò��������
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	
	return cloud;
}

// point2dTo3d ���������ͼ������ת��Ϊ�ռ�����
// input: 3ά��Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	cv::Point3f p; // 3D ��
	p.z = double(point.z) / camera.scale;
	p.x = (point.x - camera.cx) * p.z / camera.fx;
	p.y = (point.y - camera.cy) * p.z / camera.fy;
	return p;
}

// computeKeyPointsAndDesp ͬʱ��ȡ�ؼ���������������
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor)
{
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	_detector = cv::ORB::create();
	_descriptor = cv::ORB::create();

	if (!_detector || !_descriptor)
	{
		cerr << "���������Ӻͼ��������λ��! " << detector << "," << descriptor << endl;
		return;
	}

	_detector->detect(frame.rgb, frame.kp);
	_descriptor->compute(frame.rgb, frame.kp, frame.desp);

	return;
}

// estimateMotion ��������֮֡����˶�
// ���룺֡1��֡2, ����ڲ�
// �����rvec �� tvec
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	static ParameterReader pd;//����һ��������ȡ��
	vector< cv::DMatch > matches;
	cv::BFMatcher matcher;//����ƥ�䷨
	//cv::FlannBasedMatcher matcher;//�����ƥ�䷨
	matcher.match(frame1.desp, frame2.desp, matches);
	/*
	ƥ����ɺ��㷨�᷵��һЩ DMatch �ṹ���ýṹ�������¼�����Ա��
		queryIdx Դ���������ӵ�������Ҳ���ǵ�һ��ͼ�񣩡�
		trainIdx Ŀ�����������ӵ��������ڶ���ͼ��
		distance ƥ����룬Խ���ʾƥ��Խ�
	*/
	//�ж�ƥ��ĺò���
	RESULT_OF_PNP result;
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;//��С����
	double good_match_threshold = atof(pd.getData("good_match_threshold").c_str());//������ƥ����ֵ
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < minDis)
			minDis = matches[i].distance;
	}

	if (minDis < 10)
		minDis = 10;

	for (size_t i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < good_match_threshold*minDis)
			goodMatches.push_back(matches[i]);
	}

	//�����֡ƥ����������5�������һ֡
	if (goodMatches.size() <= 5)
	{
		result.inliers = -1;
		return result;
	}

	// ��һ��֡����ά��
	vector<cv::Point3f> pts_obj;
	// �ڶ���֡��ͼ���
	vector< cv::Point2f > pts_img;

	// ����ڲ�
	for (size_t i = 0; i < goodMatches.size(); i++)
	{
		// query �ǵ�һ��, train �ǵڶ���
		cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;//�ؼ��������
		// ��ȡd��ҪС�ģ�x�����ҵģ�y�����µģ�����y�����У�x���У�
		ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];//ȡ���ؼ�������
		if (d == 0)
			continue;
		pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));//�ؼ�������

		// ��(u,v,d)ת��(x,y,z)
		cv::Point3f pt(p.x, p.y, d);
		cv::Point3f pd = point2dTo3d(pt, camera);
		pts_obj.push_back(pd);
	}

	//��ƥ�����ά��Ϊ0�������һ֡
	if (pts_obj.size() == 0 || pts_img.size() == 0)
	{
		result.inliers = -1;
		return result;
	}

	//����ڲξ�������
	double camera_matrix_data[3][3] = {
		{camera.fx, 0, camera.cx},
		{0, camera.fy, camera.cy},
		{0, 0, 1}
	};

	// �����������
	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;

	/*���������Ҫ����
	һ��ƥ��õ���ά��: objectPoints
	һ���άͼ��� : imagePoints
	���صĽ������ת���� rvec ��ƽ������tvec��
	�����Ķ����㷨�еĲ�������ˣ�������Ҫ��취��������������㣬
	����ʵ���Ͼ��Ǵ�goodmatches���ȡ���ġ�*/

	// ���pnp
	cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
	//cv::solvePnP(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec);
	
	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;

	return result;
}

// cvMat2Eigen
// OpenCV ��ת��ƽ�ƾ���ת��ΪEigen�任����
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	Eigen::Matrix3d r;//��ת����
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			r(i, j) = R.at<double>(i, j);

	// ��ƽ����������ת����ת���ɱ任����
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();//�任����4*4

	Eigen::AngleAxisd angle(r);//��ת����3*1
	T = angle;
	//ƽ��������ֵ
	T(0, 3) = tvec.at<double>(0, 0);
	T(1, 3) = tvec.at<double>(1, 0);
	T(2, 3) = tvec.at<double>(2, 0);
	return T;
}

// joinPointCloud 
// ���룺ԭʼ���ƣ�������֡�Լ�����λ��
// �����������֡�ӵ�ԭʼ֡���ͼ��
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
//PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	//PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
	PointCloud::Ptr newCloud = newFrame;

	// �ϲ�����
	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*newCloud, *output, T.matrix());
	*original += *output;

	// Voxel grid �˲�������ʹ�õ��Ƹ��Ӿ���
	return original;
}

// ����index����ȡһ֡����
FRAME readFrame(int index, ParameterReader& pd)
{
	FRAME f;
	string rgbDir = pd.getData("rgb_dir");
	string depthDir = pd.getData("depth_dir");

	string rgbExt = pd.getData("rgb_extension");
	string depthExt = pd.getData("depth_extension");

	stringstream ss;
	ss << rgbDir << index << rgbExt;
	string filename;
	ss >> filename;
	f.rgb = cv::imread(filename);

	ss.clear();
	filename.clear();
	ss << depthDir << index << depthExt;
	ss >> filename;

	f.depth = cv::imread(filename, -1);
	f.frameID = index;
	return f;
}

// ����һ���˶��Ĵ�С
double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}

PointCloud::Ptr voxelDownsample(PointCloud::Ptr original)
{
	static pcl::VoxelGrid<PointT> voxel;
	static ParameterReader pd;
	double gridsize = atof(pd.getData("voxel_grid").c_str());
	voxel.setLeafSize(gridsize, gridsize, gridsize);
	voxel.setInputCloud(original);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);
	return tmp;
}
