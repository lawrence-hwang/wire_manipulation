#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs2
from std_srvs.srv import SetBool, SetBoolResponse

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

class DepthCloudTracker:
    def __init__(self, cam_spec : str = "mounted_cam", middle_elems : int = 10):
        # Subscribers to Camera
        self.depth_cam_info = rospy.Subscriber(f"/{cam_spec}/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback)
        self.segmented_depth_sub = rospy.Subscriber(f"/{cam_spec}/rscamera/depth_image/points", PointCloud2, self.segmented_depth_callback, queue_size=1)

        # Image member variables
        self.bridge = CvBridge()
        self.depth_image = []
        self.depth_cam_info = CameraInfo()
        self.intrinsics = None
        self.cam_spec = cam_spec
        self.cam_name = "Arm" if cam_spec == "arm_cam" else "Rear"

        self.x = 0
        self.y = 0
        self.end_class = None
        self.converted_depth = 0.5 # do a waitformessage?
        self.depth_adjustment = 0.05
        self.middle_elems = middle_elems

        self.converted_pt = [0.0, 0.0, 0.0]

    def convert_image_3d_point(self, depth : float):
        if self.intrinsics:
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.x, self.y], depth)  #result[0]: right, result[1]: down, result[2]: forward
            return [result[2], -result[0], -result[1]]
    
    def segmented_depth_callback(self, msg):
        try:
            sum_pt = 0.0
            num_pt = 0

            # Sort to get median x, y
            seg_pt_list = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
            sorted_x_dc = sorted(seg_pt_list, key=lambda p : p.x)
            sorted_y_dc = sorted(seg_pt_list, key=lambda p : p.y)
            n_middle_half = (self.middle_elems // 2)
            midpt = (len(sorted_x_dc) // 2) - n_middle_half
            # if len(sorted_x_dc) % 2 == 0:
            #     median_x = (sorted_x_dc[midpt - 1].x + sorted_x_dc[midpt].x) / 2
            #     median_y = (sorted_y_dc[midpt - 1].y + sorted_y_dc[midpt].y) / 2

            #     midpt = midpt - (self.middle_elems // 2)
            # else:
            #     median_x = sorted_x_dc[midpt].x
            #     median_y = sorted_y_dc[midpt].y
            x_sum = 0.0
            y_sum = 0.0
            sum_count = 0 
            for i in range(midpt, midpt + n_middle_half):
                x_sum += sorted_x_dc[i].x
                y_sum += sorted_y_dc[i].y
                sum_count += 1
            mean_x = x_sum / sum_count
            mean_y = y_sum / sum_count
            

            # Iterate through to get mean depth z
            for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                depth = pt[2]  # Depth is the Z coordinate
                sum_pt += depth
                num_pt += 1

            if num_pt > 0:
                self.converted_depth = ( sum_pt / num_pt ) + self.depth_adjustment
            
            self.converted_pt = [self.converted_depth, -mean_x, -mean_y]
        except:
            return

    def depth_cam_info_callback( self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]

        except CvBridgeError as e:
            print(e)
            return
    
    def transform_end(self, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "d435i_color_frame" if self.cam_spec == "arm_cam" else "d415_color_frame"
        t.child_frame_id = f"depth_{self.cam_spec}"

        t.transform.translation.x = self.converted_pt[0] + pos_adj[0]
        t.transform.translation.y = self.converted_pt[1] + pos_adj[1]
        t.transform.translation.z = self.converted_pt[2] + pos_adj[2]
        
        ### For now ignore rotation
        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

CAM_SPEC = "mounted_cam"
def set_cam_spec_srv(request):
    global CAM_SPEC
    # True = arm_cam, False = rear mounted_cam
    CAM_SPEC = "arm_cam" if request.data else "mounted_cam"
    return SetBoolResponse(request.data, f"Cam spec set to {CAM_SPEC}")

def main():
    rospy.init_node("depth_tracker",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)

    # CAM_SPEC = "mounted_cam"
    set_cam_spec_service = rospy.Service("/set_cam_spec", SetBool, set_cam_spec_srv)

    rear_tracker = DepthCloudTracker("mounted_cam", 10)
    arm_tracker = DepthCloudTracker("arm_cam", 10)
    while not rospy.is_shutdown():
        # print(CAM_SPEC)
        # z, x, y
        if CAM_SPEC == "arm_cam":
            arm_tracker.transform_end([-0.05, 0, 0], [0, 0, 0, 1])
        elif CAM_SPEC == "mounted_cam":
            rear_tracker.transform_end([-0.05, 0, 0.025], [0, 0, 0, 1])
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
