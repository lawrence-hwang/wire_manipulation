#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from time import sleep
from std_msgs.msg import Bool, Time
from colorama import Fore
from std_srvs.srv import SetBool

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/dlo_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

CURR_REAR_TIMESTAMP = None
def calc_active_callback(ts):
    global CURR_REAR_TIMESTAMP
    CURR_REAR_TIMESTAMP = ts

REAR_TIMESTAMP_SUB = rospy.Subscriber("/rear_timestamp", Time, calc_active_callback, queue_size=1)
    
# Client call to swap camera specification 
def set_cam_spec_service(value : Bool):
     rospy.wait_for_service("/set_cam_spec")
     try:
         set_cam_spec = rospy.ServiceProxy('/set_cam_spec', SetBool)
         response = set_cam_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def check_frame_exists(listener, source : str = "world", target : str = "line_grasp_mounted_cam"):
    """
    Check if a given frame exists
    """
    return listener.waitForTransform(source, target, rospy.Time(), rospy.Duration(1.0)) == None

def check_active_range(cam_time, interval : float = 5.0):
    # Active if difference between current time and camera time is less than the interval
    return (rospy.Time.now().to_sec() - cam_time.data.secs) < interval

def check_connector_noise(listener, parent : str = "world", child : str = "line_grasp_mounted_cam", iterations : int = 5):
    curr = listener.lookupTransform(parent, child, rospy.Time(0))
    for i in range(iterations-1):
        sleep(1)
        if curr == listener.lookupTransform(parent, child, rospy.Time(0)):
            return False # Connector frame was not updated over the duration of n iterations, meaning rear cam has lost sight
    return True # Connector frame updated over the duration of n interations, meaning rear cam has sight

#*** Node Starts Here ***#
if __name__ == "__main__":
    ### OVERALL SEARCH DEMO
    # Search algorithm is demonstrated with the following steps:
    # 0. Initiate unplugging of the connector from mock battery ORU
    # 1. Purposefully induce an engineered slip to require anomaly detection and correction
    # 2. Detect slip using Euclidean distance between the connector and grasping arm's end effector
    # 3. Flag the slip and begin searching for lost connector end with spiral search algorithm
    #    3a. Move searching arm to first search target
    #    3b. Conduct search on search target
    #    3c. If nothing found, move search target and arm and loop
    # 4. If connector found, transform frame of connector from searching arm to grasping arm and initiate retrieval
    #    Else, begin search elsewhere in environment
    # 5. Return retrieved connector end to originally intending stowing point 

    rospy.init_node('listener', anonymous=True)
    robot_control = RobotControl()

    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"
    SEARCHING_ARM   = "left"
    SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"
    arm_ids = ["left","right"]
    
    # Send object grasp robot to pre-grasp, encountering wire
    joint_goal_0 = [54, -11, -8, 105, -56, -22] # prepose start
    joint_goal_1 = [39, 6, -27, 116, -43, -31] # start
    joint_goal_unplug = [22, -6, -12, 129, -29, -40] # unplug
    joint_goal_drop = [37, 19, -61, 154, -87, -62] # drop

    joint_goal3 = [32, -7, -3, -78, 35, 74] # final stowing, maybe tune?


    listener = tf.TransformListener()
    active_duration = 5

    ### START ROUTINE for full demonstration at annual review

    ##  Initialize arms; Sleep, open grippers, and ready pose
    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots")
    for arm in arm_ids: 
        status = robot_control.move_to_target(arm, 'sleep')
        status = robot_control.set_gripper(arm, "open")

    ##  Move grasping arm to wire end and unplug
    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Moving grasping arm to connector end and unplug from battery ORU")
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_0])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_1])
    ## Grip and unplug
    status = robot_control.set_gripper(GRASPING_ARM, "close")
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_unplug])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_1])
    status = robot_control.set_gripper(GRASPING_ARM, "open")
                       
    ### START demo as wire is grasped, earliest wire could slip
    print("STATUS: Begin Scenario C4 with ")
    commands = [ # Commands from A1
        # Send to stowing point, engineering slip
        "status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_drop])",
    ]
    for command in commands:
        exec(command)
        slip_flag = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
        slip_flag = True
        if (slip_flag): # If slip detected, move arm to retrieve wire
            print("STATUS: Slip detected, initiate retrieval")
            sleep(10) # wait 5-10 real time seconds for slipped wire to settle
            status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
            status = robot_control.set_gripper(GRASPING_ARM, "open")

            rear_search = False
            arm_search = False
            # First use rear camera to detect existence and location of connector
            try:
                ## Checks that rear mounted camera has sight of the connector (from rear mounted cam perspective)
                # 1. Check that the connector frame exists 
                conn_frame_exists = check_frame_exists(listener, "world", "line_grasp_mounted_cam") # returns none if exists
                # 2. Check that the timestamp of the last active frame is within duration of current time
                curr = rospy.Time.now()
                conn_time_active = check_active_range(CURR_REAR_TIMESTAMP, active_duration)
                # 3. Check that the connector frame has been active over a given duration
                conn_update_active = check_connector_noise(listener, "world", "line_grasp_mounted_cam", active_duration) # false means swap to arm
                # Verify active checks, and throw an exception to begin arm search if non-active
                # sleep(3)
                active_flag = conn_frame_exists and conn_time_active and conn_update_active
                print(f"ACTIVE FLAG: {active_flag}")
                if active_flag == False or active_flag == True: # testing, delete this later
                    raise(tf.LookupException) # uncomment for initing search
                else:
                    rear_search = True
            
                # # If all checks pass, resume scenario wire manipulation commands
                # status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
                # status = robot_control.set_gripper(GRASPING_ARM, "open")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # print(e)
                # If rear camera fails, switch to search pattern
                print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear camera view attempt failed, initiate search routine")
                success, message = set_cam_spec_service(True) # Swap to arm cam

                ### Initiate Spiral searching
                print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiate search routine with arm cam")
                # Begin search
                success, message = set_cam_spec_service(True) # Swap to arm cam
                sleep(2.5)
                
                searchRoutine = SC.SearchRoutine("left", "right")
                arm_search = searchRoutine.search(check_subnodes=True)
    
            finally:
                if rear_search:
                    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear cam identification successful, send grasping arm for retrieval")

                    status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
                    status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_mounted_cam")
                        
                if arm_search:
                    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Arm cam search successful, send grasping arm for retrieval")

                    sleep(10)
                    status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_arm_cam_1")
                    status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_arm_cam_1")

                    # Search done, return view to rear cam
                    # success, message = set_cam_spec_service(False) # Swap back to rear cam

                # Restore connector to final pose
                status = robot_control.set_gripper(GRASPING_ARM, "close")
                status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal3])  

    ### END SCENARIO C4

    # rospy.spin()
