#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import pyproj
import numpy as np



class GpsToXYZNode(Node):
    def __init__(self):
        super().__init__('gps_to_xyz_node')
        self.subscriptionAlpha = self.create_subscription(
            NavSatFix,
            '/Alpha/fix',
            self.gps_callbackAlpha,
            10)
        self.subscriptionBob = self.create_subscription(
            NavSatFix,
            '/Bob/fix',
            self.gps_callbackBob,
            10)
        self.subscriptionCarol = self.create_subscription(
            NavSatFix,
            '/Carol/fix',
            self.gps_callbackCarol,
            10)
        # self.publisher1_ = self.create_publisher(PoseStamped, '/Alpha/xyz_topic', 10)
        self.publisherAlpha_ = self.create_publisher(PoseArray, '/Alpha/gt_xyz', 10)
        self.publisherBob_ = self.create_publisher(PoseArray, '/Bob/gt_xyz', 10)
        self.publisherCarol_ = self.create_publisher(PoseArray, '/Carol/gt_xyz', 10)

        self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:3857")
        self.beginningPoseAlpha = np.array([0.0, 0.0, 0.0])
        self.beginningPoseBob = np.array([0.0, 0.0, 0.0])
        self.beginningPoseCarol = np.array([0.0, 0.0, 0.0])
        self.poseArrayAlpha = PoseArray()
        self.poseArrayAlpha.header.frame_id = "world"
        self.poseArrayBob = PoseArray()
        self.poseArrayBob.header.frame_id = "world"
        self.poseArrayCarol = PoseArray()
        self.poseArrayCarol.header.frame_id = "world"
        self.firstGPSMessageAlpha = True
        self.firstGPSMessageBob = True
        self.firstGPSMessageCarol = True

    def gpsConversion(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        # Convert from WGS84 (EPSG:4326) to Web Mercator (EPSG:3857)
        x, y = self.transformer.transform(latitude, longitude)
        return x,y,altitude


    def gps_callbackAlpha(self, msg):
        x ,y,altitude = self.gpsConversion(msg)

        if self.firstGPSMessageAlpha:
            self.beginningPoseAlpha = np.array([x, y, altitude])
            self.firstGPSMessageAlpha = False
        else:
            point_msg = PoseStamped()
            point_msg.pose.position.x = x-self.beginningPoseAlpha[0]
            point_msg.pose.position.y = y-self.beginningPoseAlpha[1]
            point_msg.pose.position.z = 0.0#altitude-self.beginningPose[2]
            point_msg.pose.orientation.w = 1.0

            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = "world"

            self.poseArrayAlpha.poses.append(point_msg.pose)
            self.poseArrayAlpha.header.stamp = msg.header.stamp
            self.publisherAlpha_.publish(self.poseArrayAlpha)

    def gps_callbackBob(self, msg):
        x ,y,altitude = self.gpsConversion(msg)

        if self.firstGPSMessageBob:
            self.beginningPoseBob = np.array([x, y, altitude])
            self.firstGPSMessageBob = False
        else:
            point_msg = PoseStamped()
            point_msg.pose.position.x = x-self.beginningPoseBob[0]
            point_msg.pose.position.y = y-self.beginningPoseBob[1]
            point_msg.pose.position.z = 0.0#altitude-self.beginningPose[2]
            point_msg.pose.orientation.w = 1.0

            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = "world"

            self.poseArrayBob.poses.append(point_msg.pose)
            self.poseArrayBob.header.stamp = msg.header.stamp
            self.publisherBob_.publish(self.poseArrayBob)
    def gps_callbackCarol(self, msg):
        x ,y,altitude = self.gpsConversion(msg)

        if self.firstGPSMessageCarol:
            self.beginningPoseCarol = np.array([x, y, altitude])
            self.firstGPSMessageCarol = False
        else:
            point_msg = PoseStamped()
            point_msg.pose.position.x = x-self.beginningPoseCarol[0]
            point_msg.pose.position.y = y-self.beginningPoseCarol[1]
            point_msg.pose.position.z = 0.0#altitude-self.beginningPose[2]
            point_msg.pose.orientation.w = 1.0

            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = "world"

            self.poseArrayCarol.poses.append(point_msg.pose)
            self.poseArrayCarol.header.stamp = msg.header.stamp
            self.publisherCarol_.publish(self.poseArrayCarol)

def main(args=None):
    rclpy.init(args=args)
    node = GpsToXYZNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
