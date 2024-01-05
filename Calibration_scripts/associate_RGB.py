#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  3 14:58:39 2024

@author: eventcamera
"""
import numpy as np
import os
import shutil

path1 = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1/"
arr0 = os.listdir("/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam0")
arr1 = os.listdir("/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1")
arr1 = sorted(arr1)
arr0 = sorted(arr0)
path_temp = "/home/eventcamera/Eventcamera/calibration_data/RGB_stereo_event/reconstructed_event_images/cam1_temp/"


os.mkdir(path_temp)

def find_closest_integers(A, B):
    closest_integers = {}
    for b in B:
        closest = min(A, key=lambda a: abs(a - b))
        while closest in closest_integers:
            A.remove(closest)
            closest = min(A, key=lambda a: abs(a - b))
        closest_integers[closest] = b
    return closest_integers

def remove_extension_and_convert_to_int(arr):
    # Remove ".png" extension and convert to integers
    modified_arr = [int(file_name[:-4]) for file_name in arr if file_name.endswith('.png')]
    return modified_arr

arr_0 = remove_extension_and_convert_to_int(arr0)
step = (arr_0[len(arr_0)-1]-arr_0[0])/728
x = list(range(arr_0[0],arr_0[len(arr_0)-1], int(step)))
i = 0

closest = find_closest_integers(arr_0, x)


for key, value in closest.items():
    shutil.copy(path1 + str(value) + ".png", path_temp + str(key) + ".png")
    
    #os.rename(path1 + arr1[i], path0 + name)
    i = i + 1
    print(f"Integer {key} in A is closest to {value} in B.")
    