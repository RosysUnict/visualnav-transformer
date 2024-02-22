import matplotlib.pyplot as plt
import os
from typing import Tuple, Sequence, Dict, Union, Optional, Callable
import numpy as np
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

import matplotlib.pyplot as plt
import yaml

# ROS
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from utils import msg_to_pil, to_numpy, transform_images, load_model

from vint_train.training.train_utils import get_action
import torch
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


# UTILS
from topic_names import (IMAGE_TOPIC,
                        WAYPOINT_TOPIC,
                        SAMPLED_ACTIONS_TOPIC)


# CONSTANTS
#TOPOMAP_IMAGES_DIR = "../topomaps/images"
IMAGE_TOPIC = "/robot/front_rgbd_camera/rgb/image_raw"
#IMAGE_TOPIC = "/robot/front_rgbd_camera/rgb/image_throttle"
ACTION_IMAGE_TOPIC = "/trajectory"


MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"
with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"] 

# GLOBALS
context_queue = []
context_size = None  
subgoal = []

# Load the model 
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)


def callback_obs(msg):
    obs_img = msg_to_pil(msg)
    if context_size is not None:
        if len(context_queue) < context_size + 1:
            context_queue.append(obs_img)
        else:
            context_queue.pop(0)
            context_queue.append(obs_img)

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def plot_and_publish_actions_image(pred_waypoints, goal_pos, obs_img, dataset_name, display=False):
    bridge = CvBridge()
    fig, ax = plt.subplots(1,1)
    # fig, ax = plt.subplot(1,1, squeeze=False)

    start_pos = np.array([0, 0])
    if len(pred_waypoints.shape) > 2:
        trajs = [*pred_waypoints]
    else:
        trajs = [pred_waypoints]
    plot_trajs_and_points(
        ax,
        trajs,
        [start_pos, goal_pos],
        point_labels=['robot', 'goal'],
        traj_colors=[CYAN, MAGENTA],
        point_colors=[GREEN, RED],
    )
    # plot_trajs_and_points_on_image(
    #     ax[1],
    #     obs_img[0],
    #     dataset_name,
    #     trajs,
    #     [start_pos, goal_pos],
    #     traj_colors=[CYAN, MAGENTA],
    #     point_colors=[GREEN, RED],
    # )
    # ax[2].imshow(goal_img)

    # fig.set_size_inches(18.5, 10.5)
    ax.set_title(f"Action Prediction")
    # ax[1].set_title(f"Observation")
    # ax[2].set_title(f"Goal")

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


def main(args: argparse.Namespace):
    global context_size

     # load model parameters
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = model_paths[args.model]["config_path"]
    with open(model_config_path, "r") as f:
        model_params = yaml.safe_load(f)

    context_size = model_params["context_size"]

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"]
    if os.path.exists(ckpth_path):
        print(f"Loading model from {ckpth_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {ckpth_path}")
    model = load_model(
        ckpth_path,
        model_params,
        device,
    )
    model = model.to(device)
    model.eval()

    
     # load topomap
    # topomap_filenames = sorted(os.listdir(os.path.join(
    #     TOPOMAP_IMAGES_DIR, args.dir)), key=lambda x: int(x.split(".")[0]))
    # topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.dir}"
    # num_nodes = len(os.listdir(topomap_dir))
    # topomap = []
    # for i in range(num_nodes):
    #     image_path = os.path.join(topomap_dir, topomap_filenames[i])
    #     topomap.append(PILImage.open(image_path))

    #closest_node = 0
    # assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
    # if args.goal_node == -1:
    #     goal_node = len(topomap) - 1
    # else:
    #     goal_node = args.goal_node
    reached_goal = False

     # ROS
    rospy.init_node("EXPLORATION", anonymous=False)
    rate = rospy.Rate(RATE)
    image_curr_msg = rospy.Subscriber(
        IMAGE_TOPIC, Image, callback_obs, queue_size=1)
    waypoint_pub = rospy.Publisher(
        WAYPOINT_TOPIC, Float32MultiArray, queue_size=1)  
    action_traj_pub = rospy.Publisher(
        ACTION_IMAGE_TOPIC, Float32MultiArray, queue_size=1)
    sampled_actions_pub = rospy.Publisher(SAMPLED_ACTIONS_TOPIC, Float32MultiArray, queue_size=1)
    goal_pub = rospy.Publisher("/reached_goal", Bool, queue_size=1)
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    



    print("Registered with master node. Waiting for image observations...")

    if model_params["model_type"] == "nomad":
        num_diffusion_iters = model_params["num_diffusion_iters"]
        noise_scheduler = DDPMScheduler(
            num_train_timesteps=model_params["num_diffusion_iters"],
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon'
        )

    x = 1.5
    y = 1.5
    theta = 0
    # navigation loop
    # while not (rospy.is_shutdown()):
    # tf_broadcaster.sendTransform((x, y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, theta),
    #                  rospy.Time.now(),
    #                  "goal_frame",
    #                  "/robot_odom") #CHANGE TO MAP WHEN OUTDOOR
    #     #goal pose wrt static frame (x,y,theta)
    # rospy.sleep(1)
    tf_listener.waitForTransform("goal_frame", "robot_base_footprint", rospy.Time(0), rospy.Duration(4.0))
    while not rospy.is_shutdown():


        # print("Waiting for goal pose wrt robot_base_footprint")
        #decide whether to keep try catch...
        try:
            (trans,rot) = tf_listener.lookupTransform('robot_base_footprint', 'goal_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Unable to find goal_pose wrt robot_base_footprint")
            rospy.signal_shutdown()


        reached_goal = (trans[0]**2 + trans[1]**2) < 0.4
        goal_pub.publish(reached_goal)
        if reached_goal:
            print("Reached goal! Stopping...")
            rospy.signal_shutdown("Reached goal")

        chosen_waypoint = np.zeros(4)
        if (len(context_queue) > model_params["context_size"]):
                #start = max(closest_node - args.radius, 0)
                #end = min(closest_node + args.radius + 1, goal_node)
                distances = []
                waypoints = []
                batch_obs_imgs = []
                batch_goal_data = []

                #for i, sg_img in enumerate(topomap[start: end + 1]):
                transf_obs_img = transform_images(context_queue, model_params["image_size"])
                batch_obs_imgs.append(transf_obs_img)
                batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(device)

                # CHANGE goal_data = transform_images(sg_img, model_params["image_size"])
                # CHANGE batch_goal_data.append(goal_data)
                    
                # predict distances and waypoints
                # EVAL TO CHANGE batch_goal_data = torch.cat(batch_goal_data, dim=0).to(device)
                # rospy.loginfo("Angle: %s" %str(tf.transformations.euler_from_quaternion(rot)))
                # rospy.loginfo("Translation: %s" %str(trans))
                # batch_goal_data = torch.tensor([trans[0], trans[1], -tf.transformations.euler_from_quaternion(rot)[2]]).to(device)

                batch_goal_data = torch.tensor([trans[0], trans[1]]).to(device)
                batch_goal_data = batch_goal_data.unsqueeze(0)

                # start_time = time.time()
                distances, waypoints = model(batch_obs_imgs, batch_goal_data)
                # print("Time to predict: ", time.time() - start_time)
                distances = to_numpy(distances)
                waypoints = to_numpy(waypoints)

                #MODIFICARE SOTTO.. 
                # look for closest node
                #closest_node = np.argmin(distances)
                # chose subgoal and output waypoints
                # if distances[closest_node] > args.close_threshold:
                #     chosen_waypoint = waypoints[closest_node][args.waypoint]
                #     sg_img = topomap[start + closest_node]
                # else:
                #     chosen_waypoint = waypoints[min(
                #         closest_node + 1, len(waypoints) - 1)][args.waypoint]
                #     sg_img = topomap[start + min(closest_node + 1, len(waypoints) - 1)]

                chosen_waypoint = waypoints[0][args.waypoint]
                # rospy.loginfo( "Chosen Waypoint: %s"  %str(chosen_waypoint))
                # ros_image = plot_and_publish_actions_image(waypoints[0], trans[0:2], transf_obs_img[0], "recon", display=False)
                action_traj = Float32MultiArray()
                action_traj.data = waypoints[0].flatten().tolist()
                action_traj_pub.publish(action_traj)
        
        # RECOVERY MODE
        if model_params["normalize"]:
            chosen_waypoint[:2] *= (MAX_V / RATE)  
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = chosen_waypoint
        waypoint_pub.publish(waypoint_msg)

        #CHECK reached_goal = closest_node == goal_node

        # tf_broadcaster.sendTransform((x, y, 0),
        #              tf.transformations.quaternion_from_euler(0, 0, theta),
        #              rospy.Time.now(),
        #              "goal_frame",
        #              "robot_odom") #CHANGE TO MAP WHEN OUTDOOR
        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNM DIFFUSION EXPLORATION on the locobot")
    parser.add_argument(
        "--model",
        "-m",
        default="vint",
        type=str,
        help="model name (only nomad is supported) (hint: check ../config/models.yaml) (default: vint)",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    # parser.add_argument(
    #     "--dir",
    #     "-d",
    #     default="topomap",
    #     type=str,
    #     help="path to topomap images",
    # )

    #INTEGRARE
    parser.add_argument(
        "--goal-pose",
        "-gp",
        default=-1,
        type=int,
        help="""goal pose (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    # parser.add_argument(
    #     "--close-threshold",
    #     "-t",
    #     default=3,
    #     type=int,
    #     help="""temporal distance within the next node in the topomap before 
    #     localizing to it (default: 3)""",
    # )
    parser.add_argument(
        "--radius",
        "-r",
        default=4,
        type=int,
        help="""temporal number of locobal nodes to look at in the topopmap for
        localization (default: 2)""",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=8,
        type=int,
        help=f"Number of actions sampled from the exploration model (default: 8)",
    )
    args = parser.parse_args()
    print(f"Using {device}")
    main(args)


