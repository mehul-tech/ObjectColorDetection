#!/usr/bin/env python3
# encoding: utf-8
# 一个完整的 ROS2 颜色检测节点包

import os
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
from robot_cam import common # TODO
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ColorInfo, ColorsInfo
from interfaces.srv import SetColorDetectParam, SetCircleROI, SetLineROI

class ColorDetectNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.image = None
        self.running = True
        self.detect_type = {}
        self.target_colors = []
        self.weight_sum = 1.0
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.camera_type = 'Stereo'
        else:
            self.camera_type = 'Mono'

        self.image_queue = queue.Queue(maxsize=2)
        
        pkg_share = get_package_share_directory('robot_cam')
        lab_cfg = os.path.join(pkg_share, 'config', 'lab_config.yaml')
        self.lab_data = common.get_yaml_data(lab_cfg)
        self.bridge = CvBridge()
	
        pkg_share = get_package_share_directory('robot_cam')
        roi_file = os.path.join(pkg_share, 'config', 'roi.yaml')
        raw_data = common.get_yaml_data(roi_file)
        if isinstance(raw_data, dict) and len(raw_data) == 1:
            raw_data = next(iter(raw_data.values()))
        if 'ros__parameters' in raw_data:
            roi_data = raw_data['ros__parameters']
        else:
            roi_data = raw_data
        # Line ROIs
        lr = roi_data['roi_line']
        self.line_roi = {
            'roi_up': lr['roi_up'],
            'roi_center': lr['roi_center'],
            'roi_down': lr['roi_down']
        }
        # Circle ROIs
        cr = roi_data.get('roi_circle', {})
        self.circle_roi = {
            'x_min': cr.get('x_min', 0),
            'x_max': cr.get('x_max', 0),
            'y_min': cr.get('y_min', 0),
            'y_max': cr.get('y_max', 0),
        }
        # Rect ROIs
        rr = roi_data.get('roi_rect', {})
        self.rect_roi = {
            'x_min': rr.get('x_min', 0),
            'x_max': rr.get('x_max', 0),
            'y_min': rr.get('y_min', 0),
            'y_max': rr.get('y_max', 0),
        }
        
        self.camera = 'ascamera'
        self.display = self.get_parameter('enable_display').value
        self.enable_roi_display = self.get_parameter('enable_roi_display').value

        self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        self.info_publisher = self.create_publisher(ColorsInfo, '~/color_info', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # publish the image processing result

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetColorDetectParam, '~/set_param', self.set_param_srv_callback)
        self.create_service(SetLineROI, '~/set_line_roi', self.set_line_roi_srv)
        self.create_service(SetCircleROI, '~/set_circle_roi', self.set_circle_roi_srv)
        self.create_service(SetCircleROI, '~/set_rect_roi', self.set_rect_roi_srv)

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        response.success = True
        response.message = "stop"
        return response

    def set_circle_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_circle_roi")
        self.circle_roi['x_min'] = request.data.x_min
        self.circle_roi['x_max'] = request.data.x_max
        self.circle_roi['y_min'] = request.data.y_min
        self.circle_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_circle_roi"
        return response

    def set_rect_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_rect_roi")
        self.rect_roi['x_min'] = request.data.x_min
        self.rect_roi['x_max'] = request.data.x_max
        self.rect_roi['y_min'] = request.data.y_min
        self.rect_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_rect_roi"
        return response

    def set_line_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_line_roi")
        roi_up = request.data.roi_up
        roi_center = request.data.roi_center
        roi_down = request.data.roi_down
        self.line_roi['roi_up'] = [roi_up.y_min, roi_up.y_max, roi_up.x_min, roi_up.x_max, roi_up.scale]
        self.line_roi['roi_center'] = [roi_center.y_min, roi_center.y_max, roi_center.x_min, roi_center.x_max, roi_center.scale]
        self.line_roi['roi_down'] = [roi_down.y_min, roi_down.y_max, roi_down.x_min, roi_down.x_max, roi_down.scale]
        response.success = True
        response.message = "set_line_roi"
        return response

    def set_param_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_param")
        if len(request.data) == 1:
            self.target_colors = [request.data[0].color_name, ]
            self.detect_type[request.data[0].color_name] = request.data[0].detect_type
        else:
            self.target_colors = []
            for i in request.data:
                self.get_logger().info('\033[1;32m%s\033[0m' % str([i.color_name, i.detect_type]))
                self.target_colors.append(i.color_name)
                self.detect_type[i.color_name] = i.detect_type
        response.success = True
        response.message = "set_param"
        return response

    def main(self):
        while self.running:
            t1 = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            result_image = image.copy()
            h, w = image.shape[:2]
            img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # noise reduction

            centroid_sum = 0
            line_color = None

            max_area_rect = 0
            color_area_max_rect = ''
            areaMaxContour_rect = None

            max_area_circle = 0
            color_area_max_circle = ''
            areaMaxContour_circle = None
            for i in self.target_colors:
                if self.detect_type[i] == 'line' and i != '':
                    line_color = i
                    for roi in self.line_roi:
                        roi_value = self.line_roi[roi]
                        blob = img_blur[roi_value[0]:roi_value[1], roi_value[2]:roi_value[3]]
                        lab_min = tuple(self.lab_data['lab'][self.camera_type][i]['min'])
                        lab_max = tuple(self.lab_data['lab'][self.camera_type][i]['max'])
                        mask = common.lab_range_mask(blob, lab_min, lab_max)
                        mask = common.morph_cleanup(mask, ksize=3, iters=1)
                        contours = common.find_external_contours(mask, approx="TC89_L1")
                        max_contour, _ = common.get_area_max_contour(contours, threshold=0)
                        if max_contour is not None:
                            rect = cv2.minAreaRect(max_contour)
                            box = np.intp(cv2.boxPoints(rect))
                            for j in range(4):
                                box[j, 1] = box[j, 1] + roi_value[0]
                            cv2.drawContours(result_image, [box], -1, common.range_rgb[i], 2)

                            pt1_x, pt1_y = box[0, 0], box[0, 1]
                            pt3_x, pt3_y = box[2, 0], box[2, 1]
                            # the center point of the line
                            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                            cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, common.range_rgb[i], -1)  # draw the center point
                            centroid_sum += line_center_x * roi_value[-1]
                elif i != '':
                    if self.detect_type[i] == 'rect':
                        if self.enable_roi_display:
                            cv2.rectangle(result_image, (self.rect_roi['x_min'], self.rect_roi['y_min']), (self.rect_roi['x_max'], self.rect_roi['y_max']), (0, 255, 255), 1) 
                        blob = img_blur[self.rect_roi['y_min']:self.rect_roi['y_max'], self.rect_roi['x_min']:self.rect_roi['x_max']]
                    else:
                        if self.enable_roi_display:
                            cv2.rectangle(result_image, (self.circle_roi['x_min'], self.circle_roi['y_min']), (self.circle_roi['x_max'], self.circle_roi['y_max']), (0, 255, 255), 1) 
                        blob = img_blur[self.circle_roi['y_min']:self.circle_roi['y_max'], self.circle_roi['x_min']:self.circle_roi['x_max']]
                    lab_min = tuple(self.lab_data['lab'][self.camera_type][i]['min'])
                    lab_max = tuple(self.lab_data['lab'][self.camera_type][i]['max'])
                    mask = common.lab_range_mask(blob, lab_min, lab_max)
                    mask = common.morph_cleanup(mask, ksize=3, iters=1)
                    contours = common.find_external_contours(mask, approx="TC89_L1")
                    max_contour, _ = common.get_area_max_contour(contours, threshold=0)
                    if max_contour is not None:
                        area = math.fabs(cv2.contourArea(max_contour))
                        if self.detect_type[i] == 'rect':
                            if area > max_area_rect:
                                max_area_rect = area
                                color_area_max_rect = i
                                areaMaxContour_rect = max_contour
                        else:
                            if area > max_area_circle:
                                max_area_circle = area
                                color_area_max_circle = i
                                areaMaxContour_circle = max_contour
            colors_info = ColorsInfo()
            color_info_list = []
            center_pos = centroid_sum / self.weight_sum  # calculate the center point based on weighted average
            if line_color is not None and int(center_pos) != 0:
                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = line_color
                color_info.x = int(center_pos)
                color_info.y = h - 50
                color_info_list.append(color_info)
            if areaMaxContour_rect is not None:
                rect = cv2.minAreaRect(areaMaxContour_rect)
                box = np.intp(cv2.boxPoints(rect))  # four corner points of the minimum bounding rectangle
                for i in range(4):
                    box[i, 0] = box[i, 0] + self.rect_roi['x_min']
                    box[i, 1] = box[i, 1] + self.rect_roi['y_min']
                cv2.drawContours(result_image, [box], -1, common.range_rgb[color_area_max_rect], 2)  # draw the rectangle composed of the four points

                # obtain the diagonal points of the rectangle
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # center point
                x, y = int((pt1_x + pt3_x) / 2), int((pt1_y + pt3_y) / 2)

                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = color_area_max_rect
                color_info.x = x
                color_info.y = y
                color_info.angle = int(rect[2])
                color_info_list.append(color_info)
            if areaMaxContour_circle is not None:
                ((x, y), radius) = cv2.minEnclosingCircle(areaMaxContour_circle)
                x = int(x) + self.circle_roi['x_min']
                y = int(y) + self.circle_roi['y_min']
                radius = int(radius)
                cv2.circle(result_image, (x, y), radius, common.range_rgb[color_area_max_circle], 2)

                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = color_area_max_circle
                color_info.x = x
                color_info.y = y
                color_info.radius = radius
                color_info_list.append(color_info)
            colors_info.data = color_info_list
            self.info_publisher.publish(colors_info)
            if self.display:
                cv2.imshow("result", result_image)
                cv2.waitKey(1)
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
            t2 = time.time()
            t = t2 - t1
            if t < 0.03:
                time.sleep(0.03 - t)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # if the queue is full, discard the oldest image
            self.image_queue.get()
        # put the image into the queue
        self.image_queue.put(bgr_image)

def main():
    rclpy.init()
    node = ColorDetectNode('color_detect')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
