import sys


# Usage : python assocToIndLists.py <base_Directory> 

if __name__ =='__main__':

	args = sys.argv[1:]

	if (len(args) != 3):
		print "Usage : python assocToIndLists.py base_directory <0 for not_filtered and 1 for filtered> <full or keyframes>"
		sys.exit(1)
	
	base_directory = sys.argv[1]
	extra = "/" + sys.argv[3]

	fAssocName = base_directory + "/associations.txt"
	fRGBListName =  base_directory + extra + "/rgb_list.txt"
	fDepthListName = base_directory + extra + "/depth_list.txt"
	rgb_base = base_directory + "/images/rgb/"
	depth_base = base_directory + "/images/depth/"
	filtered_depth_base = base_directory + "/images/depth_filtered/"	

	fAssoc = open(fAssocName, 'r')
	fRGB = open(fRGBListName, 'w')
	fDepth = open(fDepthListName, 'w')

	num_files = int(fAssoc.readline())
	start_rgb = 1305031453.359684
	start_depth = 1305031453.359684

	if (not int(sys.argv[2]) == 0):

		for i in range(0, num_files):
			line = fAssoc.readline()
			line = line.split(' ')
			#print line[0], line[1]
			fRGB.write("{} ".format( (start_rgb+i*0.4) ) + rgb_base+line[0] + "\n")
			fDepth.write( "{} ".format( (start_depth+i*0.4) ) + filtered_depth_base + line[1].strip() + "\n")

	else:

		for i in range(0, num_files):
			line = fAssoc.readline()
			line = line.split(' ')
			#print line[0], line[1]
			fRGB.write("{} ".format( (start_rgb+i*0.4) ) + rgb_base+line[0] + "\n")
			fDepth.write( "{} ".format( (start_depth+i*0.4) ) + depth_base + line[1].strip() + "\n")