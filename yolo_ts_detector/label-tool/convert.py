#This is the convert.py
import os
from os import walk, getcwd
import glob

def convert(size, box):
	dw = 1./size[0]
	dh = 1./size[1]
	x = (box[0] + box[1])/2.0
	y = (box[2] + box[3])/2.0
	w = box[1] - box[0]
	h = box[3] - box[2]
	x = x*dw
	w = w*dw
	y = y*dh
	h = h*dh
	return (x,y,w,h)

input_path = "../traffic-sign/end-end-labels/"
out_path = "../traffic-sign/labels/"

if not os.path.exists(out_path):
	os.mkdir(out_path)

all_label_paths = glob.glob(os.path.join(input_path, '*.txt'))

""" Process """
for label_path in all_label_paths:

	""" Open input text files """
	print("Input: " + label_path)
	txt_file = open(label_path, "r")
	lines = txt_file.read().split('\n')

	""" Open output text files """
	print("Output: " + out_path + label_path.split('/')[-1])
	txt_outfile = open(out_path + label_path.split('/')[-1], "w")

	for line in lines:
		label_arr = line.split(' ')
		if(len(label_arr) == 8):
			x, y, w, h = convert((int(label_arr[6]), int(label_arr[7])),
								(int(label_arr[1]), int(label_arr[3]), int(label_arr[2]), int(label_arr[4])))
			yolo_label = "{} {} {} {} {} \n".format(label_arr[0], x, y , w, h)
			print("{} {} {} {} {}".format(label_arr[0], x, y , w, h))
			txt_outfile.write(yolo_label)

	print(" ")
	txt_outfile.close()