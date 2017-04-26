#!/usr/bin/env python
"""
classifies training data with N run majority classification

Luc Bettaieb
"""
import numpy as np
import os
import sys
import glob
import time
import random
import re

import caffe


#def main():
mean_proto_file = '/home/luc/Desktop/model/mean.binaryproto'
model_file = '/home/luc/Desktop/model/deploy.prototxt'
weights_file = '/home/luc/Desktop/model/model.caffemodel'
testimages_directory = '/home/luc/Desktop/model/4class_test'

blob = caffe.proto.caffe_pb2.BlobProto()
data = open(mean_proto_file, 'rb' ).read()
blob.ParseFromString(data)
arr = np.array( caffe.io.blobproto_to_array(blob) )
mean_file = arr[0]


pycaffe_dir = os.path.dirname(__file__)

image_dims = [300, 300]

#mean = np.load(mean_file);

#mean = np.load(args.mean_file)
mean = mean_file.mean(1).mean(1)

channel_swap = [2,1,0] #do i need this

caffe.set_mode_cpu()

classifier = caffe.Classifier(model_file, weights_file,
        image_dims=image_dims, mean=mean,
        input_scale=float, raw_scale=255.0,
        channel_swap=channel_swap)

directories = os.listdir(testimages_directory)

regex = re.compile('([-\w]+\.(?:jpg))')

for N in range(1, 10):
  for label in directories:
    cwd = testimages_directory + "/" + label
    files = os.listdir(cwd)
    images = filter(regex.match, files)

    # at this point images is the list of images of class "class"

    # choose N random images and load them into selected_images via caffe
    selected_images = []

    for i in range(0, N):
      selected_images.append(caffe.io.load_image(cwd+"/"+random.choice(images)))

    predictions = classifier.predict(selected_images, not 1)

    