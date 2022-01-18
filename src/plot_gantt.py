#!/usr/bin/env python

# This module helps creating Gantt from a dictionary or a text file.
# Output formats are a Matplotlib chart 

import sys
import random
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
from matplotlib import colors as mcolors

data = []

if len(sys.argv) != 2:
    print("Usage: " + sys.argv[0] + " filename")
else:
    # Parameters Setting
    path = sys.argv[1]
    file = open(path, 'r')

    firstLine = file.readline()
    firstLineValues = list(map(int, firstLine.split()))

    machinesNb = firstLineValues[0]
    transportersNb = firstLineValues[1]

    
    for i in range(machinesNb):
      currentLine = file.readline()
      currentLineValues = list(currentLine.split())

      machineName = "Machine " + str(currentLineValues[0])

      machineData = []

      j = 1
      while j < len(currentLineValues):
        start = int(currentLineValues[j])
        j += 1
        end = int(currentLineValues[j])
        j += 1
        name = currentLineValues[j]
        j += 1
        jobNum = int(name.split('-')[0])
        
        machineData.append({'start': start, 'end': end, 'name': name, 'jobNum': jobNum})

      machine = {'name': machineName, 'data': machineData}

      data.append(machine)

    # print(data)

    for i in range(transportersNb):
      currentLine = file.readline()
      currentLineValues = list(currentLine.split())

      machineName = "Transporter "+str(currentLineValues[0])

      machineData = []

      j = 1
      while j < len(currentLineValues):
        start = int(currentLineValues[j])
        j += 1
        end = int(currentLineValues[j])
        j += 1
        name = currentLineValues[j]
        j += 1
        jobNum = int(name.split('->')[1].split('-')[0])

        machineData.append({'start': start, 'end': end, 'name': name, 'jobNum': jobNum})

      machine = {'name': machineName, 'data': machineData}

      data.append(machine)




colors = []

for name, hex in mcolors.cnames.items():
    colors.append(name)


nb_row = len(data)

pos = np.arange(0.5, nb_row * 0.5 + 0.5, 0.5)

fig = plt.figure(figsize=(20, 8))
ax = fig.add_subplot(111)

index = 0
max_len = []

c = [random.choice(colors) for i in range(20)] #should be number of jobs

keys =[]
for machine in data:
    keys.append(machine['name'])
    for op in machine['data']:
        max_len.append(op['end'])
        clr = 'black'
        rect = ax.barh((index * 0.5) + 0.5, op['end'] - op['start'], left=op['start'], height=0.3, align='center',
                        edgecolor=clr, color=c[op['jobNum']], alpha=0.8)

        # adding label
        width = int(rect[0].get_width())
        Str = str(op['name'])
        xloc = op['start'] + 0.50 * width
        align = 'center'

        yloc = rect[0].get_y() + rect[0].get_height() / 2.0
        ax.text(xloc, yloc, Str, horizontalalignment=align,
                        verticalalignment='center', color=clr, weight='bold',
                        clip_on=True)
    index += 1

ax.set_ylim(ymin=-0.1, ymax=nb_row * 0.5 + 0.5)
ax.grid(color='gray', linestyle=':')
ax.set_xlim(0, max(10, max(max_len)))

labelsx = ax.get_xticklabels()
plt.setp(labelsx, rotation=0, fontsize=10)

locsy, labelsy = plt.yticks(pos, keys)
plt.setp(labelsy, fontsize=14)

font = font_manager.FontProperties(size='small')
ax.legend(loc=1, prop=font)

ax.invert_yaxis()

plt.title("Flexible Job Shop Solution")
plt.savefig('solution.svg')
plt.show()
