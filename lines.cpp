#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/intersections.h>
#include <math.h>

#define PI 3.14159265

pcl::PointCloud<pcl::PointXYZ> projectOntoPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f planeParams) {

	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	
	std::vector< pcl::PointXYZ , Eigen::aligned_allocator < pcl::PointXYZ > > points =  cloud->points;
	
	float a = planeParams[0];
	float b = planeParams[1];
	float c = planeParams[2];
	float d = planeParams[3];
	float t, x0, y0, z0;

	for (std::vector< pcl::PointXYZ , Eigen::aligned_allocator < pcl::PointXYZ > >::iterator i = points.begin() ; i < points.end(); i++ ){
		
		x0 = (*i).x;
		y0 = (*i).y;
		z0 = (*i).z;

		t = (-1.0) * ( a*x0 + b*y0 + c*z0 + d) / (a*a + b*b + c*c);
		tempCloud.push_back(pcl::PointXYZ( x0 + a*t , y0 + b*t, z0 + c*t));
	
	}

	pcl::PLYWriter plyWriter;
	plyWriter.write("projected.ply", tempCloud);

	return tempCloud;

}


void writePointsWithinDistance(std::vector<int> indices, pcl::PointCloud<pcl::PointXYZ> cloud, int i, std::string prodFilesDirectory){
	
	std::cout << "Writing ply file with inliers" << std::endl;

	std::string fname = prodFilesDirectory;
	fname.append("/line_ransac_");
	fname.append(std::to_string(i));
	fname.append(".ply");

	pcl::PLYWriter plyWriter;
	pcl::PointCloud<pcl::PointXYZ> tempCloud(cloud, indices);
	plyWriter.write(fname, tempCloud);
}


std::vector<Eigen::VectorXf> fitFourLines(pcl::PointCloud<pcl::PointXYZ> cloud, std::string prodFilesDirectory){

	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr lineModel(new pcl::SampleConsensusModelLine<pcl::PointXYZ>( cloud.makeShared() ));
	pcl::RandomSampleConsensus<pcl::PointXYZ> sac (lineModel, 0.02);

	std::vector<int> current_points_vec(cloud.points.size());
	std::iota(current_points_vec.begin(), current_points_vec.end(), 0);
	std::vector<Eigen::VectorXf> coeffs; 

	std::cout << "Current Size of indices : " << current_points_vec.size() << std::endl;
	//bool parallelCheckPass = true;
	float remove_thresh = 0.35;
	float inlier_thresh = 0.045;
 	for (int i = 0; i < 4 ; i ++){
		
		lineModel->setIndices(current_points_vec);
		Eigen::VectorXf coeff;
		std::vector<int> inliers;
		std::vector<int> inliers_add;
		std::vector<int> inliers_remove;
		std::vector<int> remaining_points_vec(2*current_points_vec.size());

		sac.computeModel();
		sac.getModelCoefficients (coeff);
		sac.getInliers (inliers);
		std::cout << "Inliers original " << inliers.size() << std::endl;

		lineModel->selectWithinDistance(coeff, remove_thresh, inliers_remove);
		lineModel->selectWithinDistance(coeff, inlier_thresh, inliers_add);

		std::vector<int>::iterator it = std::set_difference(current_points_vec.begin(), 
			current_points_vec.end(), inliers_remove.begin(), inliers_remove.end(), remaining_points_vec.begin());

		remaining_points_vec.resize(it - remaining_points_vec.begin());
		current_points_vec.clear();
		current_points_vec = remaining_points_vec;

		std::cout << "Remaining points after removing within dist points : " << current_points_vec.size() << std::endl;

		writePointsWithinDistance(inliers_add, cloud, i, prodFilesDirectory);

		coeffs.push_back(coeff);

	}

	return coeffs;

}

void getIntersectionPoints(std::vector<Eigen::VectorXf> coeffs, std::string prodFilesDirectory){

	for (std::vector<Eigen::VectorXf>::iterator it  =  coeffs.begin() ; it < coeffs.end(); it++){

		std::cout << "Coeffs " << *it << std::endl;
	}

	Eigen::VectorXf line1 = coeffs[0];
	Eigen::VectorXf line2 = coeffs[1];
	Eigen::VectorXf line3 = coeffs[2];
	Eigen::VectorXf line4 = coeffs[3];


	Eigen::Vector3f line1_direction(line1[3], line1[4], line1[5]);
	Eigen::Vector3f line2_direction(line2[3], line2[4], line2[5]);
	Eigen::Vector3f line3_direction(line3[3], line3[4], line3[5]);
	Eigen::Vector3f line4_direction(line4[3], line4[4], line4[5]);

	std::cout << "Dot products" << std::endl;
	std::cout << "Line 1 and 2 " << fabs( line1_direction.dot(line2_direction) ) << std::endl;
	std::cout << "Line 1 and 3 " << fabs( line1_direction.dot(line3_direction) ) << std::endl;
	std::cout << "Line 1 and 4 " << fabs( line1_direction.dot(line4_direction) ) << std::endl;
	std::cout << "Line 2 and 3 " << fabs( line2_direction.dot(line3_direction) ) << std::endl;
	std::cout << "Line 2 and 4 " << fabs( line2_direction.dot(line4_direction) ) << std::endl;
	std::cout << "Line 3 and 4 " << fabs( line3_direction.dot(line4_direction) ) << std::endl;

	std::vector<Eigen::Vector4f> intersectionPoints;
	std::vector<float> angles;
	Eigen::Vector4f intersection;
	
	if (fabs(line1_direction.dot(line2_direction)) < 0.1){

		pcl::lineWithLineIntersection(line1, line2, intersection);
		intersectionPoints.push_back(intersection);
		angles.push_back( acos( (line1_direction.dot(line2_direction)) ) * 180 / PI );

		if (fabs(line1_direction.dot(line3_direction)) < 0.1){

			pcl::lineWithLineIntersection(line1, line3, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line1_direction.dot(line3_direction)) ) * 180 / PI );
			// Then line 4 with 2 and 3 

			pcl::lineWithLineIntersection(line2, line4, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line2_direction.dot(line4_direction)) ) * 180 / PI );

			pcl::lineWithLineIntersection(line3, line4, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line3_direction.dot(line4_direction)) ) * 180 / PI );
		}
		else{

			//Else with 4

			pcl::lineWithLineIntersection(line1, line4, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line1_direction.dot(line4_direction)) ) * 180 / PI );
			//Then line 3 with 2 and 4

			pcl::lineWithLineIntersection(line2, line3, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line2_direction.dot(line3_direction)) ) * 180 / PI );

			pcl::lineWithLineIntersection(line3, line4, intersection);
			intersectionPoints.push_back(intersection);
			angles.push_back( acos( (line3_direction.dot(line4_direction)) ) * 180 / PI );

		}
	}
	// Else line1 with the 3 and 4 
	else {

		// Find the 2 intersections of line 1 first

		pcl::lineWithLineIntersection(line1, line3, intersection);
		intersectionPoints.push_back(intersection);
		angles.push_back( acos( (line1_direction.dot(line3_direction)) ) * 180 / PI );

		pcl::lineWithLineIntersection(line1, line4, intersection);
		intersectionPoints.push_back(intersection);
		angles.push_back( acos( (line1_direction.dot(line4_direction)) ) * 180 / PI );		
		// Then 2 must also intersect with 3 and 4

		pcl::lineWithLineIntersection(line2, line3, intersection);
		intersectionPoints.push_back(intersection);
		angles.push_back( acos( (line2_direction.dot(line3_direction)) ) * 180 / PI );

		pcl::lineWithLineIntersection(line2, line4, intersection);
		intersectionPoints.push_back(intersection);
		angles.push_back( acos( (line2_direction.dot(line4_direction)) ) * 180 / PI );
	}

	std::string intFileName = prodFilesDirectory;
	intFileName.append("/intersections.txt");
	std::ofstream intersectionFile (intFileName);
	
	for (int j = 0; j< 4; j++){
		std::cout << angles[j] << std::endl;
	}
	
	for (std::vector<Eigen::Vector4f>::iterator it = intersectionPoints.begin(); it < intersectionPoints.end(); it++){
		intersectionFile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;
	}
}


int main(int argc, char * argv[]){
	
	// Usage : ./lines <input-cloud name> <base directory for line point clouds and intersection file>

	if ( argc != 3 ){
		std::cout <<  "Please enter two file names" << std::endl;
	}
		
	std::string filename = std::string(argv[1]);
	std::string prodFilesDirectory = std::string(argv[2]);		
	Eigen::Vector4f planeParams(atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ> (filename , *cloud) == -1) //* load the file
  	{
    	PCL_ERROR ("Couldn't read file\n");
    	return (-1);
  	}

  	pcl::PointCloud<pcl::PointXYZ> cloud2(*cloud);

  	std::cout << (cloud->points).size() << std::endl;
  	std::cout << cloud2.points.size() << std::endl;  	

	pcl::PointCloud<pcl::PointXYZ> flatCloud = projectOntoPlane(cloud, planeParams);

	std::vector<Eigen::VectorXf> coeffs = fitFourLines(flatCloud,prodFilesDirectory );

	getIntersectionPoints(coeffs, prodFilesDirectory);

	return 0;
}
