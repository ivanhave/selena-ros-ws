#!/usr/bin/env python3
"""
Person Detection Node
Subscribes to /image_raw, runs YOLOv8 person detection,
publishes True/False to /person_detected at each frame.
Also publishes annotated image to /person_detection/image for viewing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO


class PersonDetectionNode(Node):

    def __init__(self):
        super().__init__('person_detection_node')

        # ── YOLO model ────────────────────────────────────
        self.declare_parameter('model', 'yolov8n.pt')
        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLOv8 model: {model_name}')
        self.model = YOLO(model_name)
        self.get_logger().info('YOLOv8 model loaded.')

        # ── cv_bridge ─────────────────────────────────────
        self.bridge = CvBridge()

        # ── Subscriber ────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # ── Publishers ────────────────────────────────────
        self.person_pub = self.create_publisher(
            Bool,
            '/person_detected',
            10)

        self.annotated_pub = self.create_publisher(
            Image,
            '/person_detection/image',
            10)

        self.get_logger().info('Person detection node ready.')

    def image_callback(self, msg):
        # 1. Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run YOLO detection
        results = self.model(frame, verbose=False)

        # 3. Check for person and get annotated frame
        person_detected = False
        annotated_frame = results[0].plot()  # draws boxes on frame

        for result in results:
            for box in result.boxes:
                if int(box.cls) == 0:  # class 0 = person
                    person_detected = True
                    break

        # 4. Publish detection result
        msg_out = Bool()
        msg_out.data = person_detected
        self.person_pub.publish(msg_out)

        # 5. Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if person_detected:
            self.get_logger().info('PERSON DETECTED')
        else:
            self.get_logger().debug('No person.')


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()