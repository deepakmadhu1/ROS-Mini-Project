#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'arrow_marker', 1)
        self.marker_id = 1

    def publish_marker(self, robot_location):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.frame_locked = True
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.orientation.y = 1.0

        marker.pose.position = robot_location

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.publisher_.publish(marker)
        self.get_logger().info('Published marker at robot location: (x={}, y={})'.format(robot_location.x, robot_location.y))


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera1/image_raw', self.listener_callback, 10)
        self.br = CvBridge()
        self.marker_publisher = MarkerPublisher()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data) # Convert the received image message to OpenCV format
        current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        robot_location = self.get_robot_location()  # Get the robot's location
        if robot_location is not None: # Process the image to get yellow object coordinates and publish the marker
            get_yellow_object_coordinates(current_frame_rgb, self.marker_publisher, robot_location)
        

    def get_robot_location(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())# Lookup the transform from 'map' frame to 'base_link' frame
            robot_location = transform.transform.translation
            self.get_logger().info('Robot Location - x: {}, y: {}'.format(robot_location.x, robot_location.y))
            return robot_location
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('Failed to lookup robot location')
            return None


def get_yellow_object_coordinates(image, marker_publisher, robot_location):
    
    yellow_mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255)) # Apply color thresholding to isolate yellow objects in the image
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if len(contours) > 0:
        largest_contour = contours[0]
        x, y, w, h = cv2.boundingRect(largest_contour)
        if w > 100 and h >10:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            robot_location_point = Point()
            robot_location_point.x = robot_location.x
            robot_location_point.y = robot_location.y
            robot_location_point.z = 0.0
            marker_publisher.publish_marker(robot_location_point) # Publish the marker at the robot's location
        else:
            print("Not the Yellow Object ")
    else:
        print("No yellow object found")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

