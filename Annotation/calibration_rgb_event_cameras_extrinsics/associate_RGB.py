#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  3 14:58:39 2024

@author: shrutarv awasthi
"""
import numpy as np
import os
import shutil
folder_name = 'cam0'
base_path = "/media/eventcamera/event_data/calibration/feb_18/reconstructed_event_images/"
path1 = base_path + folder_name + '_orig/'
arr0 = os.listdir(base_path + "cam1/") # RGB camera
arr1 = os.listdir(path1) # Event camera
arr1 = sorted(arr1)
arr0 = sorted(arr0)
path_temp = base_path + folder_name +  "_temp/"
threshold = 10000000

if os.path.exists(path_temp):
    shutil.rmtree(path_temp)
os.makedirs(path_temp)

def find_closest_elements(A, B):
    result = {}

    for a in A:
        closest_b = min(B, key=lambda x: abs(x - a))
        result[a] = closest_b
        B.remove(closest_b)

    return result


def remove_extension_and_convert_to_int(arr):
    # Remove ".png" extension and convert to integers
    modified_arr = [int(file_name[:-4]) for file_name in arr if file_name.endswith('.png')]
    return modified_arr

def remove_delayed_timestamps(dict, threshold):
    keys_to_remove = []
    for key, value in dict.items():
        deviation = abs(int(key) - int(value))

        if deviation > threshold:
            keys_to_remove.append(key)

    for key in keys_to_remove:
        del dict[key]
    return dict


arr_0 = remove_extension_and_convert_to_int(arr0)
arr_1 = remove_extension_and_convert_to_int(arr1)
result_dict = find_closest_elements(arr_0, arr_1)
result_dict_after_threshold = remove_delayed_timestamps(result_dict.copy(), threshold)
i = 0

for key, value in result_dict_after_threshold.items():
    print(f"Closest integer in B to {key} in A is {value}")
    src = path1 + str(value) + ".png"
    dst = path_temp + str(key) + ".png"
    shutil.copy(src, dst)
   



