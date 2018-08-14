import sys
import numpy as np

# getStripsGen.py : This file is used to obtain the intersection of a plane with a pointcloud. Variable 'eps' controls the distance from plane within which points will be selected.

# Inputs
# path to Input point cloud 
# output directory
# normal to the ground plane (args 3,4,5) 
# height from the ground at which you want to section the point cloud


def getPointsArray(inputCloud):

	f = open(inputCloud, 'r')
	
	for i in range(0,2):
		f.readline()

	numPoints = int(f.readline().split(' ')[2])

	for i in range(0, 8):
		f.readline()

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
	inputCloud =  args[0]
	outCloud = args[1]
	a = args[2]
	b = args[3]
	c = args[4]
	height = float(args[5])
	d = 1.0 - height*( sqrt(a**2 + b**2 + c**2)) 
	planeParams = np.array([a, b, c, d ] , dtype = np.float32)
	eps = 0.03

	points = getPointsArray(inputCloud)
	intersectionPoints = getPlaneCloudIntersectionPoints(planeParams, points, eps)
	writePlyFile(intersectionPoints, outCloud)
