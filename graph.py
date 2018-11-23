import matplotlib.pyplot as plt
import numpy as np

#parse data
files = ["data.txt"]
data = []
for filename in files:
  f = open(filename,"r")
  curdata = []
  line = f.readline().strip()
  while line:
    curdata.append([eval(item) for item in line.split("\t")])
    line = f.readline().strip()
  f.close()
  data.append(curdata)

# plot
fig, ax = plt.subplots(1,1)
colors = ["r","g","b","m"]
cutoff_time = 9 #milliseconds
for i in range(len(data)):
  cutoff = 0
  while(cutoff < len(data[i]) and data[i][cutoff][0] < cutoff_time):
    cutoff += 1
  x_axis = [data[i][j][0] for j in range(cutoff)]
  y_axis = [data[i][j][1] for j in range(cutoff)]
  ax.plot(x_axis,y_axis,colors[i])#,linewidth=3)

ax.set_title("Photodiode Response to LED Square Wave (duty=50%)")
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Photodiode Voltage (V)")
ax.legend(files)

fig.tight_layout()
plt.show()
