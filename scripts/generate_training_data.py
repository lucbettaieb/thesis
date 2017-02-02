#!/usr/bin/python
"""
generate_training_data.python
script to convert images in directories to different sizes

(c) Luc Bettaieb, 2017
"""

import argparse
import argcomplete
import os
import re

from PIL import Image, ImageFilter

parser = argparse.ArgumentParser(description='Process JPGs in subdirectories of a given path to a directory')
parser.add_argument('root_directory')

args = parser.parse_args()

directories = os.listdir(args.root_directory)

regex = re.compile('([-\w]+\.(?:jpg))')

sizes = [(32,32), (100,100), (200,200), (300,300)]

for directory in directories:
	cwd = args.root_directory + "/" + directory
	files = os.listdir(cwd)
	images = filter(regex.match, files)

	for size in sizes:
		save_dir = args.root_directory + "/" + directory + "_" + str(size[0])

		os.mkdir(save_dir)
	
		for img in images:
			image = Image.open(cwd + "/" + img)
			
			resized_image = image.resize(size)
			resized_image.save(save_dir + "/" + img + ".jpg", "JPEG")
