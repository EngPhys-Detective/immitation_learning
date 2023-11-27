#!/usr/bin/env python3
import os

# create a data directory in immitation_learning called data to and create 4 subdirectories
# called forward, left, right, and stop.

path = '/home/fizzer/enph_ws/src/immitation_learning/data'

if not os.path.exists(path):
    os.makedirs(path + '/forward')
    os.makedirs(path + '/left')
    os.makedirs(path + '/right')
    os.makedirs(path + '/stop')
