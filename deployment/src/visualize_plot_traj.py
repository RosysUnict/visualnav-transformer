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



from vint_train.visualizing.action_utils import plot_trajs_and_points_on_image, plot_trajs_and_points 
from vint_train.visualizing.visualize_utils import CYAN, MAGENTA, GREEN, RED
from cv_bridge import CvBridge
import cv2

import io

class VisualizeActions:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self._data_subscriber = rospy.Subscriber("/trajectory", Float32MultiArray, self.callback)
        self.action_image_pub = rospy.Publisher(
                "/action_plot", Image, queue_size=1)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def plot_and_publish_actions_image(self, pred_waypoints, display=False):  #goal_pos,
        pred_waypoints = np.array(pred_waypoints).reshape(-1, 4)
        # print(pred_waypoints)
        bridge = CvBridge()
        fig, ax = plt.subplots(1,1)
        # fig, ax = plt.subplot(1,1, squeeze=False)

        start_pos = np.array([0, 0])
        #goal_pos = start_pos
        if len(pred_waypoints.shape) > 2:
            trajs = [*pred_waypoints]
        else:
            trajs = [pred_waypoints]
        plot_trajs_and_points(
            ax,
            trajs,
            [start_pos], #, goal_pos
            point_labels=['robot'], #, 'goal'
            traj_colors=[CYAN, MAGENTA],
            point_colors=[GREEN], #, RED
        )
        ax.set_title(f"Action Prediction")
        
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        image = np.asarray(bytearray(buf.read()), dtype=np.uint8)
        buf.close()

        # Reshape the image to the appropriate size
        image = image.reshape((len(image), 1))
        image = cv2.imdecode(image, 1)
        # Convert image to ROS message
        ros_image = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        # ros_image = cv2_to_imgmsg(image)
        

        # Publish the image
        # image_pub.publish(ros_image)

        if not display:
            plt.close(fig)

        return ros_image

    def callback(self, data):
        
        ros_im = self.plot_and_publish_actions_image(data.data)#, trans[0:2])
        self.action_image_pub.publish(ros_im)

        print("Sending image...")




        


if __name__ == '__main__':
    data_vis = VisualizeActions()