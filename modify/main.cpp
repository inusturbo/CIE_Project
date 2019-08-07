//#include "slamBases.h"
#include "initKinect.h"

int main()
{
	ParameterReader pd;
	int startIndex = atoi(pd.getData("start_index").c_str());
	int endIndex = atoi(pd.getData("end_index").c_str());

	//��ʼ��
	cout << "���ڳ�ʼ�� ..." << endl;
	freenect2_ = new libfreenect2::Freenect2();
	listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);

	init();//��ʼ��Kinect

	int currIndex = startIndex;//��ǰ����
	
	PointCloud::Ptr cloud=captureImage(to_string(currIndex));
	FRAME lastFrame = readFrame(currIndex, pd);//��һ֡����
	
	// ���������ڱȽ�currFrame��lastFrame
	string detector = pd.getData("detector");
	string descriptor = pd.getData("descriptor");
	CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
	computeKeyPointsAndDesp(lastFrame, detector, descriptor);
	//PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
	

	pcl::visualization::CloudViewer viewer("viewer");
	//viewer.showCloud(cloud);
	// �Ƿ���ʾ����
	bool visualize = pd.getData("visualize_pointcloud") == string("yes");

	int min_inliers = atoi(pd.getData("min_inliers").c_str());
	double max_norm = atof(pd.getData("max_norm").c_str());

	// g2o�ĳ�ʼ��

	// ѡ���Ż�����
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3>> SlamBlockSolver;  // poseά��Ϊ 6, landmark ά��Ϊ 3
	SlamBlockSolver::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>(); // ���Է��������
	SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );      // ����������
   
	// �ݶ��½���������GN, LM, DogLeg ��ѡ
	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( blockSolver );
	// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( blockSolver );
	// g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( blockSolver );

	g2o::SparseOptimizer globalOptimizer;  // ����õľ����������
	globalOptimizer.setAlgorithm(solver);
	// ��Ҫ���������Ϣ
	globalOptimizer.setVerbose(false);

	// ��globalOptimizer���ӵ�һ������
	g2o::VertexSE3* v = new g2o::VertexSE3();
	v->setId(currIndex);
	v->setEstimate(Eigen::Isometry3d::Identity()); //����Ϊ��λ����
	v->setFixed(true); //��һ������̶��������Ż�
	globalOptimizer.addVertex(v);

	int lastIndex = currIndex; // ��һ֡��id

	for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
	{
		cout << "���ڶ�ȡ���� " << currIndex << endl;
		PointCloud::Ptr cc = captureImage(to_string(currIndex));
		FRAME currFrame = readFrame(currIndex, pd); // ��ȡcurrFrame
		computeKeyPointsAndDesp(currFrame, detector, descriptor);
		// �Ƚ�currFrame �� lastFrame
		RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
		if (result.inliers < min_inliers) //inliers������������֡
			continue;
		// �����˶���Χ�Ƿ�̫��
		double norm = normofTransform(result.rvec, result.tvec);
		cout << "norm = " << norm << endl;
		if (norm >= max_norm)
			continue;
		Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
		cout << "T=" << T.matrix() << endl;
		//cloud = joinPointCloud(cloud, currFrame, T, camera);
		cloud = joinPointCloud(cloud, cc, T, camera);

		// ��g2o�����������������һ֡��ϵ�ı�
		// ���㲿��
		// ����ֻ���趨id����
		g2o::VertexSE3 *v = new g2o::VertexSE3();
		v->setId(currIndex);
		v->setEstimate(Eigen::Isometry3d::Identity());
		globalOptimizer.addVertex(v);
		// �߲���
		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		// ���Ӵ˱ߵ���������id
		edge->vertices()[0] = globalOptimizer.vertex(lastIndex);
		edge->vertices()[1] = globalOptimizer.vertex(currIndex);
		// ��Ϣ����
		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6 >::Identity();
		// ��Ϣ������Э���������棬��ʾ���ǶԱߵľ��ȵ�Ԥ�ȹ���
		// ��ΪposeΪ6D�ģ���Ϣ������6*6���󣬼���λ�úͽǶȵĹ��ƾ��Ⱦ�Ϊ0.1�һ������
		// ��ôЭ������Ϊ�Խ�Ϊ0.01�ľ�����Ϣ����Ϊ100�ľ���
		information(0, 0) = information(1, 1) = information(2, 2) = 100;
		information(3, 3) = information(4, 4) = information(5, 5) = 100;
		// Ҳ���Խ��Ƕ����һЩ����ʾ�ԽǶȵĹ��Ƹ���׼ȷ
		edge->setInformation(information);
		// �ߵĹ��Ƽ���pnp���֮���
		edge->setMeasurement(T);
		// ���˱߼���ͼ��
		globalOptimizer.addEdge(edge);

		lastFrame = currFrame;
		lastIndex = currIndex;
		cout << "\a" << endl;
		viewer.showCloud(cloud);
		cout << "\a" << endl;
	}
	system("pause");

	// �Ż����б�
	cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
	globalOptimizer.save("result_before.g2o");
	globalOptimizer.initializeOptimization();
	globalOptimizer.optimize(100); //����ָ���Ż�����
	globalOptimizer.save("result_after.g2o");
	cout << "Optimization done." << endl;

	globalOptimizer.clear();
	pcl::io::savePCDFile( "result.pcd", *cloud );

	//�ر�Kinect
	delete freenect2_;
	delete listener_;
}