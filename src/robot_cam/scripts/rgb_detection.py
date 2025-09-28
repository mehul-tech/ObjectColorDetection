#!/usr/bin/env python3
# encoding: utf-8

import cv2
import math
import yaml
import queue
import rclpy
import threading
import numpy as np
import common # TODO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

lab_data = get_yaml_data("../config/lab_config.yaml")


def getAreaMaxContour(contours):
    """
    find the contour with the largest area.
    the parameter is a list of contours to be compared.
    """
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # iterate through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c)) 
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  
                # Only when the area is greater than 50, 
                # the contour with the largest area is valid. 
                # This allows to filter out interference
                area_max_contour = c

    return area_max_contour, contour_area_max

color_list = []
detect_color = 'None'
draw_color = range_rgb["black"]

size = (320, 240)

def run(img):
    global draw_color
    global color_list
    global detect_color
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    for i in ['red', 'green', 'blue']:
        lab_min = tuple(lab_data['lab']['Stereo'][i]['min'])
        lab_max = tuple(lab_data['lab']['Stereo'][i]['max'])
        frame_mask = common.lab_range_mask(frame_lab, lab_min, lab_max)
        frame_mask = common.morph_cleanup(frame_mask, ksize=3, iters=1)
        contours = common.find_external_contours(frame_mask, approx="TC89_L1")
        areaMaxContour, area_max = common.get_area_max_contour(contours, threshold=200)
        if areaMaxContour is not None:
            if area_max > max_area:
                max_area = area_max
                color_area_max = i
                areaMaxContour_max = areaMaxContour
    if max_area > 200:  # if the maximum area is found
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # obtain the minimum circumscribed circle
        centerX = int(common.val_map(centerX, 0, size[0], 0, img_w))
        centerY = int(common.val_map(centerY, 0, size[1], 0, img_h))
        radius = int(common.val_map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)  # draw circle

        if color_area_max == 'red':
            color = 1
        elif color_area_max == 'green':
            color = 2
        elif color_area_max == 'blue':
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:  # determine multiple times
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            if color == 1:
                detect_color = 'red'
                draw_color = range_rgb["red"]
            elif color == 2:
                detect_color = 'green'
                draw_color = range_rgb["green"]
            elif color == 3:
                detect_color = 'blue'
                draw_color = range_rgb["blue"]
            else:
                detect_color = 'None'
                draw_color = range_rgb["black"]               
    else:
        detect_color = 'None'
        draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

image_queue = queue.Queue(2)
def image_callback(ros_image):
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    bgr_image = np.array(cv_image, dtype=np.uint8)
    if image_queue.full():
        # if the queue is full, discard the oldest image
        image_queue.get()
        # put the image into the queue
    image_queue.put(bgr_image)

def main():
    running = True
    while running:
        try:
            image = image_queue.get(block=True, timeout=1)
        except queue.Empty:
            if not running:
                break
            else:
                continue
        image = run(image)
        cv2.imshow('image', image)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('color_detect_node')
    node.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', image_callback, 1)
    threading.Thread(target=main, daemon=True).start()
    rclpy.spin(node)
