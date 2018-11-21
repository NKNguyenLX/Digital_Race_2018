import glob, os
import cv2
# Percentage of images to be used for the test set
percentage_test = 10;

# Create and/or truncate train.txt and test.txt
file_train = open('../darknet/trafficsign/train.txt', 'w')  
file_test = open('../darknet/trafficsign/test.txt', 'w')

# Populate train.txt and test.txt
counter = 1  
index_test = round(100 / percentage_test) 

all_image_paths = glob.glob(os.path.join(os.getcwd(), '../traffic-sign/images/*.jpg'))


for path in all_image_paths:  
	abs_path = os.path.abspath(path)
	abs_path_arr = abs_path.split("/")
	abs_path_arr[-2] = 'images'
	abs_path = os.path.join(*abs_path_arr)
	abs_path = '/' + abs_path
	print(abs_path)
	if counter == index_test:
	    counter = 1
	    file_test.write(abs_path+'\n')
	else:
	    file_train.write(abs_path+'\n')
	    counter = counter + 1
