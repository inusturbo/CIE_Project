#include "initKinect.h"

// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;

			// 计算这个点的空间坐标
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	
	return cloud;
}

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	cv::Point3f p; // 3D 点
	p.z = double(point.z) / camera.scale;
	p.x = (point.x - camera.cx) * p.z / camera.fx;
	p.y = (point.y - camera.cy) * p.z / camera.fy;
	return p;
}

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor)
{
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	_detector = cv::ORB::create();
	_descriptor = cv::ORB::create();

	if (!_detector || !_descriptor)
	{
		cerr << "特征描述子和检测子类型位置! " << detector << "," << descriptor << endl;
		return;
	}

	_detector->detect(frame.rgb, frame.kp);
	_descriptor->compute(frame.rgb, frame.kp, frame.desp);

	return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	static ParameterReader pd;//声明一个参数读取器
	vector< cv::DMatch > matches;
	cv::BFMatcher matcher;//暴力匹配法
	//cv::FlannBasedMatcher matcher;//最近邻匹配法
	matcher.match(frame1.desp, frame2.desp, matches);
	/*
	匹配完成后，算法会返回一些 DMatch 结构。该结构含有以下几个成员：
		queryIdx 源特征描述子的索引（也就是第一张图像）。
		trainIdx 目标特征描述子的索引（第二个图像）
		distance 匹配距离，越大表示匹配越差。
	*/
	//判断匹配的好不好
	RESULT_OF_PNP result;
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;//最小距离
	double good_match_threshold = atof(pd.getData("good_match_threshold").c_str());//获得最佳匹配阈值
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

	//如果此帧匹配数量少于5则放弃这一帧
	if (goodMatches.size() <= 5)
	{
		result.inliers = -1;
		return result;
	}

	// 第一个帧的三维点
	vector<cv::Point3f> pts_obj;
	// 第二个帧的图像点
	vector< cv::Point2f > pts_img;

	// 相机内参
	for (size_t i = 0; i < goodMatches.size(); i++)
	{
		// query 是第一个, train 是第二个
		cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;//关键点的坐标
		// 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
		ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];//取出关键点的深度
		if (d == 0)
			continue;
		pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));//关键点坐标

		// 将(u,v,d)转成(x,y,z)
		cv::Point3f pt(p.x, p.y, d);
		cv::Point3f pd = point2dTo3d(pt, camera);
		pts_obj.push_back(pd);
	}

	//若匹配的三维点为0则放弃这一帧
	if (pts_obj.size() == 0 || pts_img.size() == 0)
	{
		result.inliers = -1;
		return result;
	}

	//相机内参矩阵数组
	double camera_matrix_data[3][3] = {
		{camera.fx, 0, camera.cx},
		{0, camera.fy, camera.cy},
		{0, 0, 1}
	};

	// 构建相机矩阵
	cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
	cv::Mat rvec, tvec, inliers;

	/*这个函数需要输入
	一组匹配好的三维点: objectPoints
	一组二维图像点 : imagePoints
	返回的结果是旋转向量 rvec 和平移向量tvec。
	其他的都是算法中的参数。因此，我们需要想办法构建这两组输入点，
	它们实际上就是从goodmatches里抽取来的。*/

	// 求解pnp
	cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
	//cv::solvePnP(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec);
	
	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;

	return result;
}

// cvMat2Eigen
// OpenCV 旋转和平移矩阵转换为Eigen变换矩阵
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	Eigen::Matrix3d r;//旋转矩阵
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			r(i, j) = R.at<double>(i, j);

	// 将平移向量和旋转矩阵转换成变换矩阵
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();//变换矩阵4*4

	Eigen::AngleAxisd angle(r);//旋转向量3*1
	T = angle;
	//平移向量赋值
	T(0, 3) = tvec.at<double>(0, 0);
	T(1, 3) = tvec.at<double>(1, 0);
	T(2, 3) = tvec.at<double>(2, 0);
	return T;
}

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
//PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	//PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
	PointCloud::Ptr newCloud = newFrame;

	// 合并点云
	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*newCloud, *output, T.matrix());
	*original += *output;

	// Voxel grid 滤波降采样使得点云更加均匀
	return original;
}

// 给定index，读取一帧数据
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

// 估计一个运动的大小
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
