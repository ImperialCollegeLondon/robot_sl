import rospy
import numpy as np
import cv2
from rgbd_camera_unity import RGBDCamera
from colour_segmentation import ColourSegmentation

if __name__ == '__main__':
    rospy.init_node('color_tuner', anonymous=True)
    cam = RGBDCamera('unity_d435_r', jetson=False)
    seg = ColourSegmentation([0, 0, 100], [80, 80, 255], cam.read_image, kernel_size=3, morph_op=[2,3], live_adjust=True)
    print('{}, {}'.format(seg.thresh_l, seg.thresh_h))