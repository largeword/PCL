#include <pcl/registration/ia_ransac.h>//采样一致性
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//
#include <pcl/filters/filter.h>//
#include <pcl/registration/icp.h>//icp配准
#include <pcl/registration/ndt.h>//NDT配准
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <time.h>//时间


using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{
	// 初始化点云可视化界面
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//第一个窗口显示input_cloud,第二个窗口显示target_cloud,第三个窗口显示配准之后的结果
	int v1(0), v2(0), v3(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);//第一个窗口位置、颜色
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);//第二个窗口位置以及背景颜色
	viewer->setBackgroundColor(0, 0, 0, v2);
	// viewer_final->createViewPort (0.66,0,1,1,v3);//第三个窗口位置、颜色
	// viewer_final->setBackgroundColor (0, 0, 0,v3);
	 //对待一个窗口显示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(pcd_src, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(pcd_src, input_color, "input color", v1);

	//对目标点云着色（红色）并可视化
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(pcd_tgt, 255, 0, 0);
	//对第二个视口显示
	viewer->addPointCloud<pcl::PointXYZ>(pcd_tgt, target_color, "target cloud0", v1);
	viewer->addPointCloud<pcl::PointXYZ>(pcd_tgt, target_color, "target cloud", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	//对转换后的目标点云着色（绿色）并可视化
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(pcd_final, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(pcd_final, output_color, "output cloud", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "output cloud");

	// 启动可视化
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	//等待直到可视化窗口关闭。
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//由旋转平移矩阵计算旋转角度
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}
	result_angle << ax, ay, az;

	cout << "x轴旋转角度：" << ax << endl;
	cout << "y轴旋转角度：" << ay << endl;
	cout << "z轴旋转角度：" << az << endl;
}


int main(int argc, char** argv)
{
	//加载点云文件
	PointCloud::Ptr cloud_src_o(new PointCloud);//原点云，待配准
	pcl::io::loadPCDFile("room_scan2.pcd", *cloud_src_o);
	PointCloud::Ptr cloud_tgt_o(new PointCloud);//目标点云
	pcl::io::loadPCDFile("room_scan1.pcd", *cloud_tgt_o);

	//构造变换矩阵，使目标点云旋转一定的角度成为待配准点云
	/*
	Eigen::AngleAxisf rotation(M_PI/2.85, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f translation(3.79387, 1.720047, 0);
	Eigen::Matrix4f T = (translation * rotation).matrix();
	pcl::transformPointCloud(*cloud_tgt_o, *cloud_src_o, T);
	*/

	clock_t start = clock();

	//去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;

	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;

	/*
	//下采样滤波
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.1, 0.1, 0.1);
	voxel_grid.setInputCloud(cloud_src_o);
	PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.1, 0.1, 0.1);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	PointCloud::Ptr cloud_tgt(new PointCloud);
	voxel_grid_2.filter(*cloud_tgt);
	std::cout << "down size *cloud_tgt_o from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
	*/
	PointCloud::Ptr cloud_src(new PointCloud);
	cloud_src = cloud_src_o;
	PointCloud::Ptr cloud_tgt(new PointCloud);
	cloud_tgt = cloud_tgt_o;

	//初始化正态分布变换（NDT）
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//设置依赖尺度NDT参数
	//为终止条件设置最大转换差异
	ndt.setTransformationEpsilon(0.005);
	//为More-Thuente线搜索设置最大步长
	ndt.setStepSize(0.1);
	//设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setResolution(1.0);
	//设置匹配迭代的最大次数
	ndt.setMaximumIterations(100000);
	// 设置要配准的点云
	ndt.setInputSource(cloud_src);//第二次扫描的点云进行体素滤波的结果.设置输入点云要用setInputSource而不是setInputCloud
	//设置点云配准目标
	ndt.setInputTarget(cloud_tgt);//第一次扫描的结果

	//设置使用机器人测距法得到的初始对准估计结果，可不使用
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud, init_guess);//需要指定初始变换矩阵才可以更精确配准，不指定也可以
	//ndt.align(*output_cloud);
	std::cout << "compute *cloud_tgt NDT" << endl;
	std::cout << "ndt has converged:" << ndt.hasConverged() << "  score: " << ndt.getFitnessScore() << endl;

	Eigen::Matrix4f ndt_trans;
	ndt_trans = ndt.getFinalTransformation();
	std::cout << ndt_trans << endl;

	clock_t ndt_time = clock();

	//icp配准
	PointCloud::Ptr icp_result(new PointCloud);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt_o);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(1.0);
	// 最大迭代次数
	icp.setMaximumIterations(100000);
	// 两次变化矩阵之间的差值
	icp.setTransformationEpsilon(1e-10);
	// 均方误差
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.align(*icp_result, ndt_trans);

	clock_t end = clock();
	cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "ndt time: " << (double)(ndt_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "icp time: " << (double)(end - ndt_time) / (double)CLOCKS_PER_SEC << " s" << endl;

	std::cout << "ICP has converged:" << icp.hasConverged()
		<< " score: " << icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	//cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
	std::cout << icp_trans << endl;
	//使用创建的变换对未过滤的输入点云进行变换
	pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	//保存转换的输入点云
	//pcl::io::savePCDFileASCII("_transformed_sac_ndt.pcd", *icp_result);

	//计算误差
	double a_error_x, a_error_y, a_error_z;
	double t_error_x, t_error_y, t_error_z;
	Eigen::Vector3f ANGLE_result;
	matrix2angle(icp_trans, ANGLE_result);
	cout << "计算得到的平移距离" << endl << "x轴平移" << icp_trans(0, 3) << endl << "y轴平移" << icp_trans(1, 3) << endl << "z轴平移" << icp_trans(2, 3) << endl;

	//可视化
	visualize_pcd(cloud_src_o, cloud_tgt_o, icp_result);

	return (0);
}
