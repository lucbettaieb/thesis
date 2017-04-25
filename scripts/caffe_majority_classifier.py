#!/usr/bin/env python
"""
classifies training data with N run majority classification

Luc Bettaieb
"""
import numpy as numpy
import os
import sys
import glob
import time
import random

import caffe


#def main():
mean_file = '~/Desktop/model/mean_file.npy'
model_file = '~/Desktop/model/deploy.prototxt'
weights_file = '~/Desktop/model/model.caffemodel'
testimages_directory = '~/Dekstop/model/test_images'


pycaffe_dir = os.path.dirname(__file__)

image_dims = [256, 256]

mean = np.load(mean_file);

mean = np.load(args.mean_file)
channel_swap = [2,1,0] #do i need this

caffe.set_mode_cpu()

classifier = caffe.Classifier(model_file, weights_file,
        image_dims=image_dims, mean=mean,
        input_scale=float, raw_scale=255.0,
        channel_swap=channel_swap)

directories = os.listdir(testimages_directory)

regex = re.compile('([-\w]+\.(?:jpg))')

for N in range (0, 10)
  for class in directories:
    cwd = testimages_directory + "/" + class
    files = os.listdir(cwd)
    images = filter(regex.match, files)

    # at this point images is the list of images of class "class"

    # choose N random images and load them into selected_images via caffe
    selected_images = []
    for i in range (0, N)
      selected_images.append(caffe.io.load_image(random.choice(images)))

    predictions = classifier.predict(selected_images, not args.center_only)
    