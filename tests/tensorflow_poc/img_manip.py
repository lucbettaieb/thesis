#!/usr/bin/python
"""
img_manip.py
script to convert all frames into an acceptable format for TF

(c) Luc Bettaieb, 2016
"""
import argparse
import argcomplete
import os
import re

from PIL import Image, ImageFilter

import cPickle as pickle
# scaling factor for how much to reduce image size by
scale = 20

parser = argparse.ArgumentParser(description='Process images in a directory containing JPGs')
parser.add_argument('directory')

args = parser.parse_args();

files = os.listdir(args.directory)

regex = re.compile('([-\w]+\.(?:jpg))')
images = filter(regex.match, files)

# If no images are found in the provided directory, exit.
if len(images) == 0:
	print 'WARNING: No images found in the directory provided. Exiting.'
	quit()

print 'Found images, what is their label?'
label = raw_input('Label: ')

print 'Going to process ' + str(len(images)) + ' images.'

# Make a new directory for the processed images
# if not os.path.exists(args.directory + 'processed'):
#     os.makedirs(args.directory+'processed')

images_dict = {}
label_dict = {}

sample_img = True

# Downsize the images and stick them in the label_dict
for im in images:
	str_im = im

	im = Image.open(args.directory + '/' + im)
	im_small = im.resize((im.size[0] / scale, im.size[1] / scale))

	# Apply filters here... Experiment with this!!
	# im_small = im_small.filter(ImageFilter.EDGE_ENHANCE_MORE)

	if sample_img:
		im_small.save(args.directory + '/processed/sample.jpg', "JPEG")
		sample_img = False

	px_small = im_small.load();

	pixel_list = []

	for i in xrange(0, im_small.size[0] - 1):
		for j in xrange(0, im_small.size[1] - 1):
			pixel_list.append(px_small[i, j])

	label_dict[str_im] = pixel_list

images_dict[label] = label_dict

# need to iterate over several directories....

pickle.dump(images_dict, open(args.directory + '/processed/' + label+"_training.p", "wb"))
