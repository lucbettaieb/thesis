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

parser = argparse.ArgumentParser(description='Process images in a directory containing JPGs')
parser.add_argument('directory')

args = parser.parse_args();

files = os.listdir(args.directory)

regex = re.compile('([-\w]+\.(?:jpg))')
images = filter(regex.match, files)

if len(images) == 0:
	print 'WARNING: No images found in the directory provided. Exiting.'
	quit()

print 'Going to process ' + str(len(images)) + ' images.'

if not os.path.exists(args.directory+'processed'):
    os.makedirs(args.directory+'processed')

# Downsize the images
for i in images:
	im = Image.open(args.directory + i)

	im_small = im.resize((128, 96))

	im_small.save(args.directory + 'processed/t_' + i, "JPEG")
