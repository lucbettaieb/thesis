#!/usr/bin/env python
"""
classifies training data with N run majority classification

Luc Bettaieb
"""
from __future__ import division
import matplotlib
matplotlib.use('Agg')

import numpy as np
import os
import sys
import glob
import time
import random
import re
from operator import itemgetter
import matplotlib.pyplot as plt


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
mean = mean_file.mean(1).mean(1)

pycaffe_dir = os.path.dirname(__file__)

image_dims = [256, 256]

#mean = np.load(mean_file);

#mean = np.load(args.mean_acfile)

labels_4class = ['lab', 'machineshop', 'office', 'serverroom']
labels_7class = ['circuitslab', 'entryway', 'lab', 'lounge', 'machineshop', 'office', 'serverroom']

channel_swap = [2,1,0] #do i need this

caffe.set_mode_cpu()

classifier = caffe.Classifier(model_file, weights_file,
    image_dims=image_dims, mean=mean,
    input_scale=None, raw_scale=255.0,
    channel_swap=channel_swap)

directories = os.listdir(testimages_directory)

regex = re.compile('([-\w]+\.(?:jpg))')

# for each subdirectory with the appropriate lael
for label in directories:

  #initialize accuracies for reporting
  accuracies = []

  #initialize N for reporting
  N_list = []

  #define N, where N is the number of weighted majority classifications
  N_range = 10

  for N in range(1, N_range):
    n_tests = 100
    n_correct = 0

    print "N: "+ str(N) + ", current label: " + label
    for duration in range(0, n_tests): # 100 iterations for label and N value

      cwd = testimages_directory + "/" + label
      files = os.listdir(cwd)
      images = filter(regex.match, files)

      # at this point images is the list of images of class "class"

      # choose N random images and load them into selected_images via caffe
      selected_images = []

      for i in range(0, N):
        image = cwd + "/" + random.choice(images)
        selected_images.append(caffe.io.load_image(image))

      labels = []
      caffe_ret = []

      if len(directories) == 4:
        labels = np.array(labels_4class)
        caffe_ret = [0.0, 0.0, 0.0, 0.0]
      else:
        labels = np.array(labels_7class)
        caffe_ret = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

      # generate all of the classification
      classifications = classifier.predict(selected_images, True).tolist()

      # weighted majority classification
      # add up all of the 
      for j in range (0, N):
        caffe_ret = np.add(classifications[j], caffe_ret)
      caffe_ret = np.divide(caffe_ret, N)

      max_index = caffe_ret.tolist().index(max(caffe_ret))
      # print "CURRENT LABEL: " + str(label) + " N: " + str(N)

      predictions = zip(labels, caffe_ret)

      if predictions[max_index][0] == label:
        n_correct = n_correct + 1
        print "correct!"
      else:
        print "incorrect"
      
    
    #endfor 

    accuracies.append(n_correct / n_tests)
    N_list.append(N)
    print str(n_correct) + "/" + str(n_tests)

  #endfor

  xs = np.arange(len(N_list)) 
  plt.bar(xs, accuracies, align='center')

  plt.xticks(xs, N_list) #Replace default x-ticks with xs, then replace xs with labels
  #plt.yticks(accuracies)
  plt.xlabel('N classifications')
  plt.ylabel('Accuracy')
  plt.title('Accuracy for N majority weight classifications for ' + label)

  plt.savefig('/home/luc/Desktop/model/' + label + "_testrange_" + str(N_range) + ".png")
  plt.clf()
