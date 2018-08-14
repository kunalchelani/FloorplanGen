#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>
#include <Eigen/Core>

void writePointsWithinDistance(std::vector<int> indices, pcl::PointCloud<pcl::PointXYZ> cloud, std::string fname){
	
	std::cout << "\nWriting ply file with inliers" << std::endl;

	pcl::PLYWriter plyWriter;
	pcl::PointCloud<pcl::PointXYZ> tempCloud(cloud, indices);
	plyWriter.write(fname, tempCloud);
}

void getPlanes(pcl::PointCloud<pcl::PointXYZ> cloud, Eigen::Vector3f axis, int numPlanes, std::string prodFilesDirectory) {

	std::cout << "Looking for planes" << std::endl;
	pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr planeModel(new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>( cloud.makeShared() ));
	
	//Eigen::Vector3f axis(0.0, 1.0, 0.0);
	std::cout << "Looking for planes parallel to : \n" << axis << std::endl;
	planeModel->setAxis(axis);
	planeModel->setEpsAngle(pcl::deg2rad(2.0));

	pcl::RandomSampleConsensus<pcl::PointXYZ> sac (planeModel, 0.04);

	std::vector<Eigen::VectorXf> coeffs; 
	std::vector<int> current_points_vec(cloud.points.size());
	std::iota(current_points_vec.begin(), current_points_vec.end(), 0);
	
	float fitWithinThreshold = 0.05;
	float removeWithinThreshold = 0.5;

	std::string planeParamsFileName = prodFilesDirectory;
	planeParamsFileName.append("/plane_params.txt");
	std::ofstream planeParamsFile (planeParamsFileName);
	

	for (int i =0; i < numPlanes; i++){

		Eigen::VectorXf coeff;
		std::vector<int> inliers;
		planeModel->setIndices(current_points_vec);

		sac.computeModel();
		sac.getModelCoefficients(coeff);
		sac.getInliers(inliers);
		std::cout << coeff << std::endl;

		std::vector<int> fitWithinInliers;
		std::vector<int> removeWithinInliers;
		std::vector<int> remaining_points_vec(2*current_points_vec.size());

		planeModel->selectWithinDistance(coeff, fitWithinThreshold, fitWithinInliers);
		planeModel->selectWithinDistance(coeff, removeWithinThreshold, removeWithinInliers);

		std::cout << "Size of inliers " <<  fitWithinInliers.size() << std::endl;
		for (int i = 0; i <10 ; i++)
			std::cout << fitWithinInliers[i] << std::endl;
		std::cout << "Size of points to be removed " << removeWithinInliers.size() << std::endl;	

		std::string fnameWithinInliers = prodFilesDirectory;
		fnameWithinInliers.append("/withinDistInliers_");
		fnameWithinInliers.append(std::to_string(i));
		fnameWithinInliers.append(".ply");
		
		writePointsWithinDistance(fitWithinInliers, cloud, fnameWithinInliers);

		std::vector<int>::iterator it = std::set_difference(current_points_vec.begin(), 
			current_points_vec.end(), removeWithinInliers.begin(), removeWithinInliers.end(), remaining_points_vec.begin());

		remaining_points_vec.resize(it - remaining_points_vec.begin());
		current_points_vec.clear();
		current_points_vec = remaining_points_vec;

		std::cout << "Remaining points after removing within dist points : " << current_points_vec.size() << std::endl;

		planeParamsFile << coeff[0] << " " << coeff[1] << " "  << coeff[2] << " " <<  coeff[3] << "\n" << std::endl;

	}

	planeParamsFile << axis[0] << " " << axis[1] << " "  << axis[2] << " " << 1.0 << "\n" << std::endl;

}

int main(int argc, char* argv[]){

	std::string inputDirectory = std::string(argv[1]);
	int numPlanes = std::atoi(argv[2]);
	std::string inputCloud = inputDirectory;
	inputCloud.append("/full/out_cloud.ply");
	
	std::string prodFilesDirectory = inputDirectory;
	prodFilesDirectory.append("/full/planes/");

	std::string groundPlaneFile = inputDirectory;
	groundPlaneFile.append("/full/planes/ground.txt");

	std::string planeParamsFile = inputDirectory;
	planeParamsFile.append("/full/planes/planeParams.txt");
	
	float a,b,c;

	std::ifstream fGround;
	fGround.open(groundPlaneFile.c_str());

	std::string s;
    getline(fGround, s);
    if(!s.empty()) {

    	std::stringstream ss;
    	ss << s;
    	ss >> a;
    	ss >> b;
    	ss >> c;
    }

	Eigen::Vector3f axis(a,b,c);
	axis.normalize();

	std::cout << "Axis vector : \n" << axis << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	std::cout << "Loading Pointcloud. This might take a while." << std::endl;
	if (pcl::io::loadPLYFile<pcl::PointXYZ> (inputCloud , *cloud) == -1)
  	{
    	PCL_ERROR ("Couldn't read file\n");
    	return (-1);
  	}
  	else {

  		std::cout << "Pointcloud loaded successfully" << std::endl;
  	}
	
  	//Eigen::Vector3f axis(atof(argv[3]), atof(argv[4]), atof(argv[5]));
    getPlanes(*cloud, axis, numPlanes, prodFilesDirectory);
  	std::cout << "Done! Exiting" << std::endl;	
	return 0;
}