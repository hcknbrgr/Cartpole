from os.path import exists
import numpy as np
import torch
import random
from matplotlib import pylab
import serial
import time
import struct  # for sending data as bytes
import csv
from itertools import chain
from functools import reduce

file = open('RunningLoss.csv')
data = list(csv.reader(file))
file.close()
newList = []
runningAve = 0
finalAveList = []
for a in data:
    for b in a:
        newList.append(float(b))
'''
for i in range(len(newList)):
    runningAve += newList[i]
    if (i+1) % 10 == 0:
        finalAveList.append(runningAve/10.0)
        runningAve = 0
'''
print(newList)

pylab.xlabel('Episode')
pylab.ylabel('Loss')
pylab.title('Episode Loss')
pylab.plot(newList)
pylab.savefig("LossGraph.pdf", format="pdf")
