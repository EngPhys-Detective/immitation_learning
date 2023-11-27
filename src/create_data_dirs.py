#!/usr/bin/env python3
import os

# create a data directory in immitation_learning called data to and create 4 subdirectories
# called forward, left, right, and stop.
path = '/home/fizzer/enph_ws/src/immitation_learning/data'

track = input("Enter track type (1=paved/2=dirt): ")
if track == str(1):
    track = '/paved_track'
elif track == str(2):
    track = '/dirt_track'

if not os.path.exists(path+track):
    os.makedirs(path + track + '/forward')
    os.makedirs(path + track + '/left')
    os.makedirs(path + track +'/right')
    os.makedirs(path + track + '/stop')
