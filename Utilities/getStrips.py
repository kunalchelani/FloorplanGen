import sys
import numpy as np
from scipy.cluster.vq import vq, kmeans, whiten
import matplotlib.pyplot as mlplot

def get_target_ply(filename, target_filename, height, width):

	if not filename.split('.')[1] == "ply":
		print "Please input ply file"
		return

	f = open(filename,'r')

	f.readline()
	f.readline()

	num_vertices = int(f.readline().split(' ')[2])
	print "Num vertices = {}".format(num_vertices)
	startline = []

	for j in range(0, 8):
		startline.append(f.readline())
	
	target_pts = []
	target_vals = []

	for i in range(0, num_vertices):

		line = f.readline()
		vals = line.split(' ')

		y = float(vals[1])

		if y >= height and y <= height+width:
			target_vals.append(vals)
			target_pts.append(line)

	if len(target_pts) > 0:

		print "Writing target file : " + target_filename
		write_target_ply(target_filename, startline, target_pts)

		get_clusters(target_vals)


def write_target_ply(target_filename, startline, target_pts):

	f = open(target_filename, 'w')
	f.write("ply\n");
	f.write("format ascii 1.0\n")
	f.write("element vertex {}\n".format(len(target_pts)))
	
	for i in range(0, len(startline)):
		f.write(startline[i])

	for i in range(0, len(target_pts)):
		f.write(target_pts[i])

def get_clusters(target_vals):

	target_array = np.array(target_vals, dtype=np.float32)
	
	X = (target_array[:,0]).reshape(-1,1)
	Y = (target_array[:,1]).reshape(-1,1)
	Z = (target_array[:,2]).reshape(-1,1)

	mlplot.hist(X, 20)
	mlplot.show()


'''	
	x_vals = np.zeros(len(target_vals))
	y_vals = np.zeros(len(target_vals))
	z_vals = np.zeros(len(target_vals))

	for i in range(0, len(target_vals)):
		x_vals.append(target_vals[i][0])
'''


#def plot_clusters()

if __name__ == '__main__':

	width = 0.05

	arguments = sys.argv[1:]
	
	if len(arguments) < 2:
		print "provide source and target"
		sys.exit(1)

	source_ply = sys.argv[1]
	target_ply = sys.argv[2]
	
	print source_ply
	print target_ply

	for i in range(-4, 5):
		
		print i*0.5
		print "Getting ply_file_ready"
		
		target_filename = target_ply+"{}".format(i*0.5)+".ply"

		get_target_ply(source_ply, target_filename, i*0.5, width)


