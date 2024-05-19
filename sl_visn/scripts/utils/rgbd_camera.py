import cv2
import numpy as np

import rospy
from ros_numpy import numpify
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

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
        self.image_topic_compressed = f'/{camera_name}/color/image_raw/compressed'
        self.info_topic = f'/{camera_name}/aligned_depth_to_color/camera_info'
        self.frame = f'/{camera_name}_color_optical_frame'
        self.camera_info = rospy.wait_for_message(self.info_topic, CameraInfo).K
        self.img_msg = None
        self.depth_msg = None
        rospy.Subscriber(self.image_topic_compressed, CompressedImage, self.img_sub, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_sub, queue_size=1)

    def img_sub(self, msg):
        self.img_msg = msg

    def depth_sub(self, msg):
        self.depth_msg = msg

    def read_image(self):
        while self.img_msg is None:
            rospy.sleep(0.01)
        img_msg = self.img_msg
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        if self.jetson:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            h = img[:,:,0]
            s = img[:,:,1]
            v = img[:,:,2]
            img = cv2.merge([cv2.add(h, 5),cv2.add(s, -10),cv2.add(v, 10)])
            img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        if self.roi:
            img = img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]
        self.img_msg = None
        return img

    def read_image_ori(self):
        img_msg = rospy.wait_for_message(self.image_topic, Image)
        img = numpify(img_msg)
        if self.jetson:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            h = img[:,:,0]
            s = img[:,:,1]
            v = img[:,:,2]
            img = cv2.merge([cv2.add(h, 5),cv2.add(s, -10),cv2.add(v, 10)])
            img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if self.roi:
            img = img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]
        return img

    def read_depth(self):
        while self.depth_msg is None:
            rospy.sleep(0.01)
        depth_msg = self.depth_msg
        depth = numpify(depth_msg)
        self.depth_msg = None
        return depth

    def read_camera_info(self):
        return self.camera_info

if __name__ == "__main__":
    realsense_camera = RealsenseCamera()
