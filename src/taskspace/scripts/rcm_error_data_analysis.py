#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
import matplotlib.pyplot as plt
 

plt.figure()

errors = np.genfromtxt("rcm_error_data.csv", delimiter=",").astype(float)[:,1]
histdata = np.histogram(errors, bins='auto')

# The error data will have many values that will be constant when the robot foes not move, these should be removed, in order to not skew the data
max_bin_size = np.max(histdata[0])
max_bin_index = np.where(histdata[0]==max_bin_size)[0][0]
remove_from = histdata[1][max_bin_index]
remove_to = histdata[1][max_bin_index+1]
cleaned_errors = [e for e in errors if e < remove_from or e > remove_to]

avg = np.average(cleaned_errors)
std = np.std(cleaned_errors)
print("Cleaned errors: avg=",avg, "std=",std)

removed_errors = [e for e in errors if e >= remove_from and e <= remove_to]
avg_removed = np.average(removed_errors)
std_removed = np.average(removed_errors)
print("Removed errors: avg=",avg_removed, "std=",std_removed)
print("total_point=", len(errors), "cleaned_points=", len(cleaned_errors), "removed_points=", len(removed_errors))

plt.subplot(1, 3, 1)
plt.hist(errors, bins='auto', color='g')
plt.hist(cleaned_errors, bins='auto')
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))

plt.subplot(1, 3, 2)
plt.hist(cleaned_errors, bins='auto')
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))

plt.subplot(1, 3, 3)
plt.hist(removed_errors, bins='auto', color='g')
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))

# plt.yscale('log')
plt.show()