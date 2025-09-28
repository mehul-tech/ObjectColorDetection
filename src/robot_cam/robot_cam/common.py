#!/usr/bin/env python3
# encoding: utf-8

import math
import yaml
from rclpy.node import Node
# from geometry_msgs.msg import Pose, Quaternion

## === === ===  Student TODO Block (Activity 1)  === === ===
# 目标：实现颜色检测最小管线的 4 个纯函数（只用 OpenCV/NumPy，不引入 ROS）。
# 评分口径：能在 rgb_detection.py 的画面里把目标颜色圈出来并显示颜色名即可。
# 小提示：
# - 这四个函数会被 rgb_detection.py 和 color_detect_node.py 复用；
# - 不写逐像素 for 循环（性能差且容易错），尽量用 OpenCV/NumPy；
# - 输入/输出的 dtype、形状要和注释一致，避免后续崩溃。

import cv2
import numpy as np
from typing import List, Tuple

def lab_range_mask(img_lab: np.ndarray,
                   lab_min: Tuple[int,int,int],
                   lab_max: Tuple[int,int,int]) -> np.ndarray:
    """在 LAB 空间做区间阈值，返回 0/255 二值图。
    输入：
      img_lab: (H,W,3), 已由上游 BGR→LAB
      lab_min/lab_max: 三元组(0..255)，需满足 min[i] ≤ max[i]
    返回：
      mask: (H,W), uint8，命中像素=255，其余=0
    """
    raise NotImplementedError

def morph_cleanup(mask: np.ndarray, ksize: int = 3, iters: int = 1) -> np.ndarray:
    """形态学去噪：矩形核(ksize×ksize) 做 Opening(腐蚀→膨胀)。
    输入：
      mask: (H,W) uint8 的 0/255 图
      ksize: 奇数（默认3）；iters: 迭代次数（默认1）
    返回：
      清理后的 mask（仍为 0/255, uint8）
    """
    raise NotImplementedError

def find_external_contours(mask: np.ndarray, approx: str = "TC89_L1"):
    """查找外轮廓。
    输入：mask 为 0/255 二值图
    参数：approx='TC89_L1' → cv2.CHAIN_APPROX_TC89_L1；其他→ cv2.CHAIN_APPROX_NONE
    返回：contours 列表（OpenCV 4: contours, hierarchy = cv2.findContours(...)）
    """
    raise NotImplementedError

def get_area_max_contour(contours: List[np.ndarray], threshold: int = 0):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the
    largest contour, otherwise return None)
    """
    raise NotImplementedError
    # return area_max_contour, contour_area_max

# === End Student TODO Block ===


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
