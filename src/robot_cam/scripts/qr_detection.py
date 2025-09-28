#!/usr/bin/env python3
# encoding: utf-8

import os
import queue
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class QRCodeDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)
        self.enable_display = True
        self.resize_to = None  # (640, 480) or None
        self.qr_roi = None # (x0, y0, x1, y1) or None
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)

    # === === === Student TODO Block (Activity #1: QR) === === ===

    # Implement these 3 functions. No ROS dependencies required here.
    from typing import List, Dict, Tuple
    Detection = Dict[str, object]

    @staticmethod
    def qr_preprocess(bgr: np.ndarray,
                      roi: Tuple[int,int,int,int] = None,
                      resize_to: Tuple[int,int] = None) -> np.ndarray:
        """
        Preprocess image for QR decoding.
        """
        # Apply ROI crop if given
        if roi is not None:
            x0, y0, x1, y1 = roi
            bgr = bgr[y0:y1, x0:x1]

        # Resize if requested
        if resize_to is not None:
            bgr = cv2.resize(bgr, resize_to, interpolation=cv2.INTER_LINEAR)

        # Convert to grayscale for QR detection (better contrast)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # Optional: mild histogram equalization to improve contrast
        gray = cv2.equalizeHist(gray)

        return gray

    @staticmethod
    def qr_decode(image_for_decode: np.ndarray) -> List[Detection]:
        """
        Decode QR codes from preprocessed image.
        """
        detections: List[QRCodeDetectNode.Detection] = []
        detector = cv2.QRCodeDetector()

        # detectAndDecodeMulti returns lists of decoded strings, points, and straight_qrcodes
        retval, decoded_infos, points, _ = detector.detectAndDecodeMulti(image_for_decode)

        if retval and points is not None:
            for data, pts in zip(decoded_infos, points):
                if pts is None or len(pts) < 4:
                    continue
                polygon = np.int32(pts.reshape(-1, 2))
                area = cv2.contourArea(polygon)
                detections.append({
                    'data': data,
                    'polygon': polygon,
                    'area': float(area),
                })

        return detections

    @staticmethod
    def annotate_and_select(bgr_canvas: np.ndarray, detections: List[Detection]) -> Tuple[np.ndarray, Detection]:
        """
        Draw polygons for all detections and select the primary QR by max area.
        """
        primary = None
        max_area = -1.0

        for det in detections:
            poly = det['polygon']
            data = det['data']
            area = det['area']

            # Draw polygon outline
            cv2.polylines(bgr_canvas, [poly], isClosed=True, color=(0, 255, 0), thickness=2)

            # Put decoded text near the first vertex
            if isinstance(data, str) and data:
                x, y = poly[0]
                cv2.putText(bgr_canvas, data, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Update primary selection
            if area > max_area:
                max_area = area
                primary = det

        return bgr_canvas, primary

    
    # === === === End Student TODO Block === === ===

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            
            # Preprocessing
            try:
                proc = self.qr_preprocess(image, roi=self.qr_roi, resize_to=self.resize_to)
            except Exception as e:
                self.get_logger().warn(f"qr_preprocess failed: {e}")
                proc = image

            # 2) QR Decoding
            try:
                detections = self.qr_decode(proc)  # {'data','polygon','area'}
            except Exception as e:
                self.get_logger().warn(f"qr_decode failed: {e}")
                detections = []

            # 3) Annotation and Selection of Primary QR Code
            try:
                annotated, primary = self.annotate_and_select(image.copy(), detections)
            except Exception as e:
                self.get_logger().warn(f"annotate_and_select failed: {e}")
                annotated, primary = image, None

            # Log primary data if available
            if primary and isinstance(primary.get('data'), str) and primary['data']:
                self.get_logger().info(primary['data'])


            if self.enable_display:
                try:
                    cv2.imshow('qr', annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), 27):
                        self.running = False
                        break
                except Exception:
                    self.enable_display = False

        if self.enable_display:
            cv2.destroyAllWindows()

def main():
    node = QRCodeDetectNode('qrcode_detect')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()
