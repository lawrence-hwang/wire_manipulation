#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
from time import sleep
from std_msgs.msg import Bool, Time
from colorama import Fore
from std_srvs.srv import SetBool

# from dual_robot_msgs.srv import *
from robot_services import RobotControl

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
# This is here because I had issues importing and sourcing, but ideally we want a line like above.
# Not important to fix but could be cleaner (undergrad/intern task idea?)
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/dlo_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

sample_subscriber = rospy.Subscriber("/rear_timestamp", Time, calc_active_callback, queue_size=1)
    
def example_service_call(value : Bool):
     rospy.wait_for_service("/set_cam_spec")
     try:
         set_cam_spec = rospy.ServiceProxy('/set_cam_spec', SetBool)
         response = set_cam_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def misc_commands(listener, source : str = "world", target : str = "line_grasp_mounted_cam"):
    # Check if a given frame exists by checking if the transform exists between two frames
    active1 = listener.waitForTransform(source, target, rospy.Time(), rospy.Duration(1.0)) == None
    active2 = listener.lookupTransform(source, target, rospy.Time(0)) # alternative option

#*** Node Script Starts Here ***#
if __name__ == "__main__":
    # Start with ROS syntax for initiating the node
    rospy.init_node('listener', anonymous=True)
    # Our messy import from earlier gives us RobotControl, which is an object we can define and use to give
    # movement commands to the robot arms. This is done through interfacing with MoveIt (see RobotControl source code)
    robot_control = RobotControl()

    # Defining strings to make referring to arms, and their assigned tasks clear through script.
    # Example here from my demo script
    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"
    SEARCHING_ARM   = "left"
    SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"
    arm_ids = ["left","right"]
    
    """
    ARM MOVEMENT COMMANDS
    0. Syntax
        We specify a command for arm movement like so:
            status = robot_control.[]
        - status is the return from a movement, which specifies the execution result and any system log messages
        - robot_control is the movement commander defined earlier
        - [] is where we specify the movement command, outlined below
    1. Move arm by providing a joint goal specified by a list of joint angles
        Useful for when moving arm to a specific position. 
        Simply supply a list of joint angles (easily found in RViz) to the command, like so:
            joint_goal = [...]
            status = robot_control.move_to_joint_goal(ARM_NAME, [x * np.pi / 180 for x in joint_goal])
        Bit of math is necessary to convert joint angles to quaternions usable by the movement commanding function.
    2. Move arm to predefined pose
        Some poses are predefined and can be navigated to by simply referring to its name.
        For the arms, these are 'sleep' and 'ready'.
        For the gripper, these are 'open' and 'close'.
        More can be added and defined, however this process is lost to me as it was done by Ian. I believe you need the MoveIt wizard. (another good undergad task?)
        Execute like so:
            status = robot_control.move_to_target(arm, 'sleep')
            status = robot_control.set_gripper(arm, "open")
    3. Move arm to a target frame
        The gripper's frame (a/b_ee_arm_link) can be maneuvered such that it matches that of a frame in the world. 
        This is crux of my work and how I primarily moved the arm: essentially defining and moving a frame around the world
        and sending the arm to it using the following command:
            status = robot_control.move_to_frame(ARM_NAME, "frame_name")
        Simply specify the arm you'd like to move followed by a string containing the name of the frame. This then tasks
        the motion planner to position the gripper end effector frame exactly where the frame is. This worked for my
        research as it was applicable to simple pick/place tasks, but this approach lacks definition of the rest of the arm. The
        motion planners we use are undeterministic (see probablistic roadmap motion planners), meaning every (including repeated)
        trajectories to the same target pose may vary, so keep this in mind when using this approach and navigating around obstacles,
        wanting the arm to move a certain way, etc.
    """
    # Example of 1: Movement by joint goal
    joint_goal_example = [54, -11, -8, 105, -56, -22]
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_example])

    # Example of 2: Movement by predefined pose
    status = robot_control.move_to_target(SEARCHING_ARM, 'sleep')
    status = robot_control.set_gripper(SEARCHING_ARM, "open")
    
    # Example of 3: Movement by frame target
    status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
    
    ### Miscellaneous commands
    # 0. Print messages to console log in color
    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Example log message")
    # 1. Wait for flag or message from the rest of the system before executing action
    message = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
    """
    Can be used like:
    if (message == True):
        ...
    """
    # 2. Example of calling service; defined as function above. Service is technically defined in a different file,
    # it just gets called by the function we define and THAT is then called here to use it.
    success, message = example_service_call(True)
                
    # Uncomment to keep node alive and looping; currently skipped as we just run through script
    # rospy.spin()
