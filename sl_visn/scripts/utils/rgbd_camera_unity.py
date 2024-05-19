import cv2
import numpy as np

import rospy
from ros_numpy import numpify
from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PoseStamped

class RGBDCamera:
    def __init__(self, camera_name, roi=None, depth_thresh=None, jetson=False):
        '''
        roi: (x1, y1, x2, y2)
        '''
        self.roi=roi
        self.depth_thresh = depth_thresh # cap image and depth image by depth threshold
        self.jetson = jetson
        self.image_topic = f'/{camera_name}/color/image_raw'
        self.depth_topic = f'/{camera_name}/aligned_depth_to_color/image_raw'
        self.info_topic = f'/{camera_name}/color/camera_info'
        self.pose_topic = f'/{camera_name}/pose'
        self.frame = f'/{camera_name}_color_optical_frame'
        self.camera_info = rospy.wait_for_message(self.info_topic, CameraInfo).K
        self.img_msg = None
        self.depth_msg = None
        self.pose_msg = None
        rospy.Subscriber(self.image_topic, Image, self.img_sub, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_sub, queue_size=1)
        # rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_sub, queue_size=1)

    def img_sub(self, msg):
        self.img_msg = msg

    def depth_sub(self, msg):
        self.depth_msg = msg

    # def pose_sub(self, msg):
    #     self.pose_msg = msg

    def read_image(self):
        while self.img_msg is None and not rospy.is_shutdown():
            rospy.sleep(0.01)
        img_msg = self.img_msg

        reshaped = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        img = np.flipud(reshaped)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if self.roi:
            img = img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]
        self.img_msg = None
        return img

    def read_depth(self):
        while self.depth_msg is None and not rospy.is_shutdown():
            rospy.sleep(0.01)
        depth_msg = self.depth_msg
        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape((depth_msg.height, depth_msg.width))
        # flip and rescale to millimeters
        depth = np.flipud(depth)*1000

        # add gaussian noise
        # depth += np.random.normal(0, 3, depth.shape) # d435 noise: 2% of distance

        self.depth_msg = None
        return depth

    # def read_camera_pose(self):
    #     pose = np.concatenate([numpify(self.pose_msg.pose.position), numpify(self.pose_msg.pose.orientation)])
    #     self.pose_msg = None
    #     return pose

    def read_camera_info(self):
        return self.camera_info

if __name__ == "__main__":
    rospy.init_node("rgbd_cam_wrapper")
    rgbd_camera = RGBDCamera("unity_d435_r")
    img = rgbd_camera.read_depth()
    cv2.imshow("test", img)
    cv2.waitKey()
    img = rgbd_camera.read_image()
    cv2.imshow("test", img)
    cv2.waitKey()
