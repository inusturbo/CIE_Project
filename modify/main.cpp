//#include "slamBases.h"
#include "initKinect.h"

int main()
{
	ParameterReader pd;
	int startIndex = atoi(pd.getData("start_index").c_str());
	int endIndex = atoi(pd.getData("end_index").c_str());

	//初始化
	cout << "正在初始化 ..." << endl;
	freenect2_ = new libfreenect2::Freenect2();
	listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);

	init();//初始化Kinect

	int currIndex = startIndex;//当前索引
	
	PointCloud::Ptr cloud=captureImage(to_string(currIndex));
	FRAME lastFrame = readFrame(currIndex, pd);//上一帧数据
	
	// 我们总是在比较currFrame和lastFrame
	string detector = pd.getData("detector");
	string descriptor = pd.getData("descriptor");
	CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
	computeKeyPointsAndDesp(lastFrame, detector, descriptor);
	//PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
	

	pcl::visualization::CloudViewer viewer("viewer");
	//viewer.showCloud(cloud);
	// 是否显示点云
	bool visualize = pd.getData("visualize_pointcloud") == string("yes");

	int min_inliers = atoi(pd.getData("min_inliers").c_str());
	double max_norm = atof(pd.getData("max_norm").c_str());

	// g2o的初始化

	// 选择优化方法
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3>> SlamBlockSolver;  // pose维度为 6, landmark 维度为 3
	SlamBlockSolver::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>(); // 线性方程求解器
	SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );      // 矩阵块求解器
   
	// 梯度下降方法，从GN, LM, DogLeg 中选
	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( blockSolver );
	// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( blockSolver );
	// g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( blockSolver );

	g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
	globalOptimizer.setAlgorithm(solver);
	// 不要输出调试信息
	globalOptimizer.setVerbose(false);

	// 向globalOptimizer增加第一个顶点
	g2o::VertexSE3* v = new g2o::VertexSE3();
	v->setId(currIndex);
	v->setEstimate(Eigen::Isometry3d::Identity()); //估计为单位矩阵
	v->setFixed(true); //第一个顶点固定，不用优化
	globalOptimizer.addVertex(v);

	int lastIndex = currIndex; // 上一帧的id

	for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
	{
		cout << "正在读取数据 " << currIndex << endl;
		PointCloud::Ptr cc = captureImage(to_string(currIndex));
		FRAME currFrame = readFrame(currIndex, pd); // 读取currFrame
		computeKeyPointsAndDesp(currFrame, detector, descriptor);
		// 比较currFrame 和 lastFrame
		RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
		if (result.inliers < min_inliers) //inliers不够，放弃该帧
			continue;
		// 计算运动范围是否太大
		double norm = normofTransform(result.rvec, result.tvec);
		cout << "norm = " << norm << endl;
		if (norm >= max_norm)
			continue;
		Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
		cout << "T=" << T.matrix() << endl;
		//cloud = joinPointCloud(cloud, currFrame, T, camera);
		cloud = joinPointCloud(cloud, cc, T, camera);

		// 向g2o中增加这个顶点与上一帧联系的边
		// 顶点部分
		// 顶点只需设定id即可
		g2o::VertexSE3 *v = new g2o::VertexSE3();
		v->setId(currIndex);
		v->setEstimate(Eigen::Isometry3d::Identity());
		globalOptimizer.addVertex(v);
		// 边部分
		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		// 连接此边的两个顶点id
		edge->vertices()[0] = globalOptimizer.vertex(lastIndex);
		edge->vertices()[1] = globalOptimizer.vertex(currIndex);
		// 信息矩阵
		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6 >::Identity();
		// 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
		// 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
		// 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
		information(0, 0) = information(1, 1) = information(2, 2) = 100;
		information(3, 3) = information(4, 4) = information(5, 5) = 100;
		// 也可以将角度设大一些，表示对角度的估计更加准确
		edge->setInformation(information);
		// 边的估计即是pnp求解之结果
		edge->setMeasurement(T);
		// 将此边加入图中
		globalOptimizer.addEdge(edge);

		lastFrame = currFrame;
		lastIndex = currIndex;
		cout << "\a" << endl;
		viewer.showCloud(cloud);
		cout << "\a" << endl;
	}
	system("pause");

	// 优化所有边
	cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
	globalOptimizer.save("result_before.g2o");
	globalOptimizer.initializeOptimization();
	globalOptimizer.optimize(100); //可以指定优化步数
	globalOptimizer.save("result_after.g2o");
	cout << "Optimization done." << endl;

	globalOptimizer.clear();
	pcl::io::savePCDFile( "result.pcd", *cloud );

	//关闭Kinect
	delete freenect2_;
	delete listener_;
}