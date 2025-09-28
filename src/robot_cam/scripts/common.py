#!/usr/bin/env python3
# encoding: utf-8

import math
import yaml
from rclpy.node import Node
# from geometry_msgs.msg import Pose, Quaternion

# === === === Student TODO Block (Activity #1: Color) === === ===
import cv2
import numpy as np
from typing import List, Tuple

def lab_range_mask(img_lab: np.ndarray,
                   lab_min: Tuple[int,int,int],
                   lab_max: Tuple[int,int,int]) -> np.ndarray:
    """
    Produce binary mask (0/255) where LAB pixels fall in [lab_min, lab_max].
    """
    lab_min_np = np.array(lab_min, dtype=np.uint8)
    lab_max_np = np.array(lab_max, dtype=np.uint8)
    mask = cv2.inRange(img_lab, lab_min_np, lab_max_np)
    return mask

def morph_cleanup(mask: np.ndarray, ksize: int = 3, iters: int = 1) -> np.ndarray:
    """
    Denoise binary mask with morphological opening + closing.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    # Opening removes small noise, Closing fills tiny holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iters)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iters)
    return mask

def find_external_contours(mask: np.ndarray, approx: str = "TC89_L1"):
    """
    Find external contours of the binary mask.
    """
    if approx == "TC89_L1":
        method = cv2.CHAIN_APPROX_TC89_L1
    elif approx == "TC89_KCOS":
        method = cv2.CHAIN_APPROX_TC89_KCOS
    else:
        method = cv2.CHAIN_APPROX_SIMPLE

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, method)
    return contours

def get_area_max_contour(contours: List[np.ndarray], threshold: int = 0):
    """
    Return contour with largest area above threshold.
    """
    best_contour = None
    best_area = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area > best_area and area >= threshold:
            best_area = area
            best_contour = c
    return best_contour, float(best_area)


# === === === End Student TODO Block === === ===


range_rgb = {
    'red': (0, 50, 255),
    'blue': (255, 50, 0),
    'green': (50, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def loginfo(msg):
    Node.get_logger().info('\033[1;32m%s\033[0m' % msg)

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def empty_func(img=None):
    return img

def set_range(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)
    return data

def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()
