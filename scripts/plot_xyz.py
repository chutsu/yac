#!/usr/bin/env python3
""" Plot XYZ """
import sys
import pandas
import proto
import matplotlib.pylab as plt

csv_data = pandas.read_csv(sys.argv[1])
key_time = sys.argv[2]
key_x = sys.argv[3]
key_y = sys.argv[4]
key_z = sys.argv[5]
label_y = "Displacement [m]"
data = {"Estimates": csv_data}
line_styles = ["."]
proto.plot_xyz("", data, key_time, key_x, key_y, key_z, label_y, line_styles=line_styles)
plt.show()
