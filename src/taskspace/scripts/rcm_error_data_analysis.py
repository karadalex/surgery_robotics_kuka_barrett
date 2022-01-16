#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
import matplotlib.pyplot as plt
 

errors = np.genfromtxt("rcm_error_data.csv", delimiter=",")
histdata = np.histogram(errors[:,1], bins='auto')
# plt.hist(errors[:,1], bins='auto')

# The error data will have many values that will be constant when the robot foes not move, these should be removed, in order to not skew the data
max_bin_size = np.max(histdata[0])
max_bin_index = np.where(histdata[0]==max_bin_size)[0][0]
remove_from = histdata[1][max_bin_index]
remove_to = histdata[1][max_bin_index+1]
cleaned_errors = [e for e in errors[:,1] if e < remove_from or e > remove_to]

avg = np.average(cleaned_errors)
std = np.std(cleaned_errors)
print(avg, std)

plt.hist(cleaned_errors, bins='auto')
plt.show()