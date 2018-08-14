import cv2
from os import listdir
from os.path import isfile, join
import sys

if __name__ == '__main__':
	
	sourceDirec = sys.argv[1]
	targetDirec = sys.argv[2]
	sourceFiles = [f for f in listdir(sourceDirec) ]
	
	for file in sourceFiles:
		img = cv2.imread(sourceDirec+file, -1)
		gfilt = cv2.GaussianBlur(img, (5,5), 0)
		cv2.imwrite(targetDirec+file,gfilt)

