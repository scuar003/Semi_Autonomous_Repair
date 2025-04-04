import os
from dotenv import load_dotenv

load_dotenv()

ROS_DOMAIN_ID = os.getenv("ROS_DOMAIN_ID", "0")
NODE_CONFIG = {
    "turtlesim": os.getenv("TURTLESIM_CMD", "ros2 run turtlesim turtlesim_node"),
    "realsense": os.getenv("REALSENSE_CMD", "ros2 launch realsense_camera rs_launch.py"),
    "ur_robot": os.getenv("UR_ROBOT_CMD", "ros2 launch ur_robot_driver ur_control.launch.py")
}
