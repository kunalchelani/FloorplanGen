import sys
import numpy as np
import math

# getStripsGen.py : This file is used to obtain the intersection of a plane with a pointcloud. Variable 'eps' controls the distance from plane within which points will be selected.

# Inputs
# path to Input point cloud 
# output directory
# normal to the ground plane (args 3,4,5) 
# height from the ground at which you want to section the point cloud


def getPointsArray(inputCloud):

	f = open(inputCloud, 'r')
	st = [""]
	while (not st[0] == "element"):
		st = f.readline().split(" ")

	numPoints = int(st[2])
	print numPoints
	while (not f.readline().strip() == "end_header"):
		continue

	print "out"
	points = np.zeros( (4, numPoints))

	for i in range(0, numPoints):
		k = np.array( f.readline().split(' ')[0:3], dtype = np.float32)
		points[0:3, i] = ( k )

	points[3, :] = np.ones(numPoints)
	return points 

def getPlaneCloudIntersectionPoints(planeParams, points, eps):

	intersectionPoints = []
	
	for i in range(points.shape[1]):

		if abs(np.dot(planeParams, points[:, i])) < eps :
			
			intersectionPoints.append(points[:, i])

	return intersectionPoints

def writePlyFile(points, outCloud):

	numPoints = len(points)

	f = open(outCloud, 'w')
	f.write("ply\n")
	f.write("format ascii 1.0\n")
	f.write("element vertex {}\n".format(numPoints))
	f.write("property float x\nproperty float y\nproperty float z\n")
	f.write("end_header\n")
	for i in range(0, numPoints):
		f.write("{} {} {}\n".format( points[i][0], points[i][1], points[i][2] ))

if __name__ == '__main__':

	args = sys.argv[1:]
	
	inputDirectory = args[0]
	height = float(args[1])

	inputCloud =  inputDirectory + "/full/out_cloud.ply"
	outCloud = inputDirectory + "/full/strips/strip_{}".format(height) + ".ply"
	groundParamsFile = inputDirectory + "/full/planes/ground.txt"

	groundParams = open(groundParamsFile, "r")
	nums = groundParams.readline().split(" ")
	print nums[0]

	a = float(nums[0])
	b = float(nums[1])
	c = float(nums[2])

	

	d = 1.0 - height*( math.sqrt(a**2 + b**2 + c**2)) 
	planeParams = np.array([a, b, c, d ] , dtype = np.float32)
	eps = 0.03

	print "Getting points into an array"
	points = getPointsArray(inputCloud)

	print "Finding points at intersection of plane and room pointcloud"
	intersectionPoints = getPlaneCloudIntersectionPoints(planeParams, points, eps)
	
	print "Writing ply file"	
	writePlyFile(intersectionPoints, outCloud)
