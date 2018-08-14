from os import listdir
from os.path import isfile, join
import sys

def func(str):
	fp = str.split('.')
	return int(fp[0][5:])

if __name__ == '__main__':

	args = sys.argv[1:]
	
	if (len(args) != 1):
		print "Please enter the base_directory"
		sys.exit(1)

	base_directory = sys.argv[1] #example "/home/floorplan/mmcr"
	rgb_path = base_directory + "images/rgb" #example "/home/cvlab/extracted_test1/rgb"
	depth_path = base_directory + "images/depth" #example "/home/cvlab/extracted_test1/depth"
	associations_file = base_directory + "/associations.txt"

	rgb_file_names = [f for f in listdir(rgb_path) ]
	depth_file_names = [f for f in listdir(depth_path) ]

	rgb_file_names.sort(key=func)
	depth_file_names.sort(key=func)

	if not len(rgb_file_names) == len(depth_file_names):
		print "Unequal number of rgb and depth images, please check !"
		exit
	
	'''
	if len(rgb_file_names) > 10000:
		
		for i in range(0, len(rgb_file_names)):

			fp = rgb_file_names[i].split('.')
			
			if ( len(fp[0]) == 9 ):

				rgb_file_names[i] = "Color" + '0' + fp[0][5:] + '.'+ fp[1]
				depth_file_names = "Depth" + 
	'''
	num_files = len(rgb_file_names)

	f1 = open(associations_file, 'w')

	f1.write("{} \n".format(num_files))

	for i in range(0, num_files):
		f1.write(rgb_file_names[i] + " " + depth_file_names[i] +"\n")
 
