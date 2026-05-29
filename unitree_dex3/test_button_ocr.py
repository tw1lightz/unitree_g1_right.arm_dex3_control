#!/usr/bin/env python3
"""Standalone test: detect and recognize hand-drawn elevator buttons on paper.

Uses PaddleOCR to find text regions and read digits. No YOLO model needed.
Works with the G1 RealSense camera or a static image file."""

import argparse
import cv2
import numpy as np
from paddleocr import PaddleOCR


def run_ocr_on_image(image, ocr):
    """Run PaddleOCR and return detected text regions with content."""
    results = ocr.ocr(image, cls=True)
    detections = []
    if not results or not results[0]:
        return detections
    for line in results[0]:
        box = line[0]          # [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
        text = line[1][0]      # recognized string
        confidence = line[1][1] # confidence score
        detections.append({
            'box': box,
            'text': text,
            'confidence': confidence,
        })
    return detections


def draw_detections(image, detections):
    """Draw bounding boxes and text labels on the image."""
    display = image.copy()
    for det in detections:
        box = np.array(det['box'], dtype=np.int32)
        # Draw polygon bounding box
        cv2.polylines(display, [box], True, (0, 255, 0), 2)
        # Draw filled polygon for label background
        x_coords = box[:, 0]
        y_coords = box[:, 1]
        xmin, xmax = int(x_coords.min()), int(x_coords.max())
        ymin, ymax = int(y_coords.min()), int(y_coords.max())
        label = f"{det['text']} ({det['confidence']:.2f})"
        # Put text above the box
        cv2.putText(display, label, (xmin, ymin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return display


def capture_from_realsense():
    """Capture one frame from the G1 RealSense camera via ROS2."""
    import rclpy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    captured = None

    def callback(msg):
        nonlocal captured
        bridge = CvBridge()
        captured = bridge.imgmsg_to_cv2(msg, 'bgr8')

    rclpy.init()
    node = rclpy.create_node('button_test_capture')
    sub = node.create_subscription(
        Image, '/camera/camera/color/image_raw', callback, 10)

    print("Waiting for RealSense image... Press Ctrl+C when captured.")
    try:
        # Spin until we get one frame
        while captured is None:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return captured


def main():
    parser = argparse.ArgumentParser(
        description='Test elevator button OCR on hand-drawn buttons')
    parser.add_argument('--image', type=str, default=None,
                        help='Path to image file. If not set, captures from RealSense.')
    parser.add_argument('--save', type=str, default=None,
                        help='Save result image to this path.')
    parser.add_argument('--lang', type=str, default='en',
                        help='OCR language (default: en)')
    args = parser.parse_args()

    # Get image
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            print(f"Failed to read image: {args.image}")
            return
    else:
        image = capture_from_realsense()
        if image is None:
            print("No image captured from RealSense.")
            return

    print(f"Image size: {image.shape[1]}x{image.shape[0]}")

    # Initialize PaddleOCR
    print("Initializing PaddleOCR...")
    ocr = PaddleOCR(use_angle_cls=True, lang=args.lang, show_log=False, use_gpu=False)

    # Run OCR
    print("Running OCR...")
    detections = run_ocr_on_image(image, ocr)

    if not detections:
        print("No text detected. Try:")
        print("  1. Drawing buttons with thicker, clearer numbers")
        print("  2. Ensuring good lighting (avoid glare)")
        print("  3. Making numbers large enough (at least 1cm height)")
    else:
        print(f"Detected {len(detections)} text regions:")
        for det in detections:
            print(f"  '{det['text']}' (confidence: {det['confidence']:.2f})")

    # Display result
    display = draw_detections(image, detections)
    cv2.imshow('Elevator Button OCR Test', display)
    print("Press any key to close, 's' to save...")
    key = cv2.waitKey(0)
    if key == ord('s') and args.save:
        cv2.imwrite(args.save, display)
        print(f"Saved to {args.save}")
    elif key == ord('s'):
        default_save = '/home/unitree/Desktop/button_test_result.jpg'
        cv2.imwrite(default_save, display)
        print(f"Saved to {default_save}")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()