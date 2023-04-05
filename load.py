import matplotlib.pyplot as plt
import numpy as np
import math
import time
import sys
from queue import PriorityQueue
import cv2 as cv
from matplotlib import transforms
import json


with open('rpm.json', 'r') as openfile:
    rpm = json.load(openfile)

with open('coords.json', 'r') as openfile:
    path = json.load(openfile)

for i in range(0, len(path)-1):
    xi = path[i][0]
    yi = path[i][1]
    xn = path[i+1][0]
    yn = path[i+1][1]
    v = np.sqrt((xn-xi)**2+(yn-yi)**2)

    r = 0.038
    L = 0.354
    rz = (r/L)*(rpm[i][1]-rpm[i][0])
    print(v/100)
    print(rz/100)
    print(" ")
    time.sleep(1)
