## This python program can automatically draw boxes
## around every bit transmitted for our protocol.
## We used this program for debugging problems like
## if our receiver is out of alignment with our transmitter.

import matplotlib.pyplot as plt
import numpy as np
import sys


## NETWORK CONFIGURATIONS
ppm_slot = 2.5
beacon_dur = 20*ppm_slot
ppm_count = 2
max_packet = 64

#parse data
filename = sys.argv[1]
cutoff = None
beacon_start = None

## Parse CLI parameters
if len(sys.argv) >= 3:
  cutoff = float(sys.argv[2])
if len(sys.argv) >= 4:
  beacon_start = float(sys.argv[3])

## Read data file
data = []
f = open(filename,"r")
line = f.readline().strip()
while line:
  data.append([eval(item) for item in line.split("\t")])
  line = f.readline().strip()
f.close()

# Plot file data
fig, ax = plt.subplots(1,1)
x_axis = [data[i][0] for i in range(len(data))]
y_axis = [data[i][1] for i in range(len(data))]
ax.plot(x_axis,y_axis)

## show high voltage cutoff line
if cutoff is not None:
  ax.axhline(y=cutoff, color = 'r')

## draw boxes around each part of our frame
if beacon_start is not None:
  #draw beacon
  time = beacon_start
  ax.axvline(x=time, color = 'r', linewidth=2)
  time += beacon_dur
  ax.axvline(x=time, color = 'r', linewidth=2)

  #draw boxes around each PPM bit
  while(time < beacon_start+beacon_dur+ppm_slot*ppm_count*8*max_packet):
    ax.axvline(x=time+ppm_slot, color = 'r', linewidth=0.5)
    time += ppm_slot*ppm_count
    ax.axvline(x=time, color = 'r', linewidth=2)
  
## display the results
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Photodiode Voltage (V)")
fig.tight_layout()
plt.show()
