#!/usr/bin/env python
import matplotlib.pyplot as plt
import yaml

# ROS
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from utils import msg_to_pil, to_numpy, transform_images, load_model

from vint_train.training.train_utils import get_action
from PIL import Image as PILImage
import numpy as np
import argparse
import yaml
import time


import tf
import geometry_msgs.msg
from nav_msgs.msg import Path


class VisualizePath:
    def __init__(self):
        rospy.loginfo("VisualizePath Started")
        rospy.init_node('listener', anonymous=True)

        self._data_subscriber = rospy.Subscriber("/trajectory", Float32MultiArray, self.callback)
        self.action_image_pub = rospy.Publisher(
                "/vint_path", Path, queue_size=1)
        # spin() simply keeps python from exiting until this node is stopped

        rospy.loginfo("VisualizePath Spinning")
        rospy.spin()



    def callback(self, data):
        path_msg = Path()
        pred_waypoints = np.array(data.data).reshape(-1, 4)
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "robot_base_footprint"
        for i in range(len(pred_waypoints)):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x = pred_waypoints[i][0]
            pose.pose.position.y = pred_waypoints[i][1]
            pose.pose.position.z = 0
            angle = np.arctan2(pred_waypoints[i][3], pred_waypoints[i][2])
            orientation = tf.transformations.quaternion_from_euler(0, 0, angle)
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]
            path_msg.poses.append(pose)
        self.action_image_pub.publish(path_msg)





        


if __name__ == '__main__':
    data_vis = VisualizePath()