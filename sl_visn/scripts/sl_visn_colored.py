import cv2
import time
import numpy as np
import open3d as o3d
from math import isnan, pi, atan2, sqrt

import rospy
from tf import TransformListener
from tf.transformations import euler_from_quaternion, compose_matrix, quaternion_from_euler
np.float = np.float64
from ros_numpy import msgify
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
from sl_msgs.srv import *

from utils.point_clouds import depth_pixel_to_metric_coordinate, read_point_from_region, read_points_from_region, xy_to_yx, euclidian_distance
from utils.colour_segmentation import ColourSegmentation

class shoelacingVision:
    debug = False
    processed_frame_topic = '/sl_visn/frame_processed'
    find_target_srv = 'find_targets'
    robot_frame = "yumi_base_link"
    l515_roi = [0, 180, 1280, 720] # 128[0, 0, 125], [110, 110, 0]0*720

    def __init__(self, auto_execution, sim):
        # read ROS parameters
        self.robot_name = rospy.get_param("~robot_name", "unity")
        self.aglet_thresh_blue_low = rospy.get_param("~aglet_thresh_blue_low", [90, 0, 0])
        self.aglet_thresh_blue_high = rospy.get_param("~aglet_thresh_blue_high", [5, 80, 80])
        self.aglet_thresh_red_low = rospy.get_param("~aglet_thresh_red_low", [0, 0, 90])
        self.aglet_thresh_red_high = rospy.get_param("~aglet_thresh_red_high", [80, 80, 5])
        self.eyelet_thresh_blue_low = rospy.get_param("~eyelet_thresh_blue_low", [90, 0, 0])
        self.eyelet_thresh_blue_high = rospy.get_param("~eyelet_thresh_blue_high", [5, 80, 80])
        self.eyelet_thresh_red_low = rospy.get_param("~eyelet_thresh_red_low", [0, 0, 80])
        self.eyelet_thresh_red_high = rospy.get_param("~eyelet_thresh_red_high", [80, 80, 5])

        self.auto_execution = auto_execution
        if sim:
            from utils.rgbd_camera_unity import RGBDCamera
        else:
            from utils.rgbd_camera import RGBDCamera
        self.l515 = RGBDCamera(self.robot_name+'_l515', roi=self.l515_roi, jetson=True)
        self.d435_l = RGBDCamera(self.robot_name+'_d435')
        self.d435_r = RGBDCamera(self.robot_name+'_d435_r')

        # initialise the tf listener
        self.listener = TransformListener()

        # get segementation instances
        rospy.loginfo("Set colour threshold for the blue aglet")
        self.seg_aglet_b = ColourSegmentation(self.aglet_thresh_blue_low, self.aglet_thresh_blue_high, 
                                           self.l515.read_image, live_adjust=not auto_execution)
        rospy.loginfo(f'Blue aglet thresholds: {self.seg_aglet_b.thresh_l}, {self.seg_aglet_b.thresh_h}')
        rospy.loginfo("Set colour threshold for the red aglet")
        self.seg_aglet_r = ColourSegmentation(self.aglet_thresh_red_low, self.aglet_thresh_red_high,
                                            self.l515.read_image, live_adjust=not auto_execution)
        rospy.loginfo(f'Red aglet thresholds: {self.seg_aglet_r.thresh_l}, {self.seg_aglet_r.thresh_h}')
                
        rospy.loginfo("Set colour threshold for the blue eyelets")
        self.seg_eyelet_b = ColourSegmentation(self.eyelet_thresh_blue_low, self.eyelet_thresh_blue_high,
                                                self.d435_l.read_image, live_adjust=not auto_execution)
        rospy.loginfo(f'Blue eyelets thresholds: {self.seg_eyelet_b.thresh_l}, {self.seg_eyelet_b.thresh_h}')
        rospy.loginfo("Set colour threshold for the red eyelets")
        self.seg_eyelet_r = ColourSegmentation(self.eyelet_thresh_red_low, self.eyelet_thresh_red_high,
                                                self.d435_r.read_image, live_adjust=not auto_execution)
        rospy.loginfo(f'Red eyelets thresholds: {self.seg_eyelet_r.thresh_l}, {self.seg_eyelet_r.thresh_h}')

        # define the publishers
        self.frame_pub = rospy.Publisher(self.processed_frame_topic, Image, queue_size=10)
        if self.debug:
            self.edge_pub = rospy.Publisher('sl_visn/edge', Image, queue_size=10)
            self.mask_pub = rospy.Publisher('sl_visn/masks', Image, queue_size=10)
            self.aglet_pub_a = rospy.Publisher("sl_visn/aglet_a", PoseArray, queue_size=1)
            self.aglet_pub_b = rospy.Publisher("sl_visn/aglet_b", PoseArray, queue_size=1)
            self.eyelet_pub = rospy.Publisher("a", PoseArray, queue_size=1)
            self.eyelet_pub_r = rospy.Publisher("b", PoseArray, queue_size=1)

        # define the services
        rospy.Service(self.find_target_srv, findTargetsService, self.srvPubTargets)
        rospy.loginfo('Find targets service ready')

        if self.debug:
            rospy.sleep(1)
            self.find_targets = rospy.ServiceProxy(self.find_target_srv, findTargetsService)
            rospy.wait_for_service(self.find_target_srv)
            request = findTargetsServiceRequest()
            request.target_name = 'aglet_b' # eyelets_b, eyelets_r, aglet_a, aglet_b
            request.camera_name = 'l515' # d435_l, d435_r, l515
            while not rospy.is_shutdown():
                start = time.perf_counter()
                response = self.find_targets(request)
                stop = time.perf_counter()
                print(stop-start)
                rospy.sleep(0.5)

    ''' Callback of find target service '''
    def srvPubTargets(self, request):
        # generate an empty response
        response = findTargetsServiceResponse()
        # turn the img msg to numpy array
        if request.camera_name == 'l515':
            img = self.l515.read_image()
            depth = self.l515.read_depth()
            camera_intrinsics = self.l515.read_camera_info()
            (trans,quat) = self.listener.lookupTransform(self.robot_frame, self.l515.frame, rospy.Time(0))
            cam_to_rob = compose_matrix(angles=euler_from_quaternion(quat), translate=trans)
        elif request.camera_name == 'd435_l':
            camera = self.d435_l
            seg = self.seg_eyelet_b
        elif request.camera_name == 'd435_r':
            camera = self.d435_r
            seg = self.seg_eyelet_r
        else:
            rospy.logerr("Unknown type of camera!")
            return response

        # start processing the request
        if request.target_name == "aglet_a":
            # localise the two markers from the masks
            pose_list = []
            img_list = []
            for _ in range(5):
                img = self.l515.read_image()
                depth = self.l515.read_depth()
                # get masks for two markers
                mask = self.seg_aglet_r.predict_img(img)
                pose, img = self.aglet_detection(img, mask, depth, cam_to_rob, camera_intrinsics)
                if pose is not None:
                    pose_list.append(pose)
                    img_list.append(img)
            if len(pose_list)>0:
                pose_list = np.array(pose_list)
                id = pose_list[:,2].argsort()[pose_list.shape[0]//2]
                pose = pose_list[id]
                img = img_list[id]

                # generate the response message
                target = PoseArray()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = self.robot_frame
                aglet_pose = Pose()
                aglet_pose.position = Point(*pose[:3])
                aglet_pose.orientation = Quaternion(*pose[3:])
                target.poses.append(aglet_pose)
                response.target = target

                if self.debug:
                    self.aglet_pub_a.publish(target) 

        if request.target_name == "aglet_b":
            # localise the two markers from the masks
            pose_list = []
            img_list = []
            for _ in range(5):
                img = self.l515.read_image()
                depth = self.l515.read_depth()
                # get masks for two markers
                mask = self.seg_aglet_b.predict_img(img)
                pose, img = self.aglet_detection(img, mask, depth, cam_to_rob, camera_intrinsics)
                if pose is not None:
                    pose_list.append(pose)
                    img_list.append(img)

            if len(pose_list)>0:
                pose_list = np.array(pose_list)
                id = pose_list[:,2].argsort()[pose_list.shape[0]//2]
                pose = pose_list[id]
                img = img_list[id]

                # generate the response message
                target = PoseArray()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = self.robot_frame
                aglet_pose = Pose()
                aglet_pose.position = Point(*pose[:3])
                aglet_pose.orientation = Quaternion(*pose[3:])
                target.poses.append(aglet_pose)
                response.target = target

                if self.debug:
                    self.aglet_pub_b.publish(target) 

        if request.target_name == "eyelets_r":
            eyelets, img, confidence = self.eyelet_detection(camera, seg)
            # generate the target point messages
            target = PoseArray()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = self.robot_frame
            for eyelet in eyelets:
                eyelet_pose = Pose()
                eyelet_pose.position = Point(*eyelet[:3])
                normal = eyelet[3:]
                if any(np.isnan(normal)):
                    quaternion = [0,0,0,1]
                else:
                    if normal[1]<0: normal = [ -v for v in normal] # force to face the shoe centre

                    roll = 0 # aglet has 2 degree of rotational freedom
                    yaw = atan2(normal[1], normal[0]) 
                    # if yaw<0: yaw += pi # only defined to the left side 
                    pitch = -atan2(normal[2], sqrt(normal[0]**2+normal[1]**2))
                    # pitch = 0 # set pitch to 0 to avoid bumping into the sheo tongue during pulling after the insertion
                    # quaternion = [roll, pitch, yaw, 0]
                    quaternion = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
                eyelet_pose.orientation = Quaternion(*quaternion)
                target.poses.append(eyelet_pose)
            response.target = target

            conf_msg = Float64MultiArray()
            conf_msg.data = confidence
            response.confidence = conf_msg

            if self.debug:
                self.eyelet_pub_r.publish(target)   

        if request.target_name == "eyelets_b":
            eyelets, img, confidence = self.eyelet_detection(camera, seg)
            # generate the target point messages
            target = PoseArray()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = self.robot_frame
            for eyelet in eyelets:
                eyelet_pose = Pose()
                eyelet_pose.position = Point(*eyelet[:3])
                normal = eyelet[3:]
                if any(np.isnan(normal)):
                    quaternion = [0,0,0,1]
                else:
                    if normal[1]>0: normal = [ -v for v in normal]

                    roll = 0 # aglet has 2 degree of rotational freedom
                    yaw = atan2(normal[1], normal[0]) 
                    # if yaw>0: yaw -= pi # only defined to the left side 
                    pitch = atan2(-normal[2], sqrt(normal[0]**2+normal[1]**2))
                    # pitch = 0 # set pitch to 0 to avoid bumping into the shoe tongue during pulling after the insertion
                    quaternion = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
                    # quaternion = [roll, pitch, yaw, 0]
                eyelet_pose.orientation = Quaternion(*quaternion)
                target.poses.append(eyelet_pose)
            response.target = target

            conf_msg = Float64MultiArray()
            conf_msg.data = confidence
            response.confidence = conf_msg

            if self.debug:
                self.eyelet_pub.publish(target)   

        if request.target_name == "shoe":
            pose, img = self.shoe_detection(img, depth, cam_to_rob, camera_intrinsics)
            if pose is not None:
                # generate the response message
                target = PoseArray()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = self.robot_frame
                shoe_pose = Pose()
                shoe_pose.position = Point(*pose[:3])
                shoe_pose.orientation = Quaternion(*pose[3:])
                target.poses.append(shoe_pose)
                response.target = target

        # publish a new frame
        self.frame_pub.publish(msgify(Image, cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 'rgb8'))
        return response

    def aglet_detection(self, img, mask, depth, transform, camera_intrinsics):
        '''
        [Main recognition function to detect the aglets]
        Input: image
        output: pose of the aglet (7), result image
        '''
        # remove the shoe region
        _, width = np.shape(mask)
        mask[:, width//5*2:width//5*3] = 0
        # mask[530:600, 405:870] = 0 # remove the central region

        # smooth the mask
        # mask = cv2.GaussianBlur(mask,(5,5),0)
        
        if self.debug:
            self.mask_pub.publish(msgify(Image, mask, 'mono8'))

        # hierarchy: [Next, Previous, First_Child, Parent]
        contours_temp, hierachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_temp is None or hierachy is None:
            return None, img
        
        # filter out all the non first layer contours
        contours = []
        for id, h in enumerate(hierachy[0]):
            if h[3] == -1:
                contours.append(contours_temp[id])
        # img = cv2.drawContours(img, contours, -1, (255,255,0), 3)

        # filter out the big contours
        boxes = []
        sizes = []
        for i in range(len(contours)):
            box = cv2.minAreaRect(contours[i]) #(center(x, y), (width, height), angle of rotation)
            size = box[1][0]*box[1][1]
            if size>=1000: # ignore the big contours
                continue
            if size<=5: # ignore the small contours
                continue
            boxes.append(box)
            sizes.append(size)
            
        # find the two boxes whose distance fits requirement
        dist_mat = np.eye(len(boxes))*1000
        candidates = []
        for i in range(len(boxes)):
            for j in range(i):
                distance = euclidian_distance(boxes[i][0], boxes[j][0])
                dist_mat[i,j] = dist_mat[j,i] = distance
                if 10<=distance<=70:
                    dist_mat[i,j] = dist_mat[j,i] = 1000 # not needed anymore
                    candidates.append([i, j])
        ids = None
        for c in candidates:
            if all(dist_mat[c[0], :]>20) and all(dist_mat[c[1], :]>20):
                ids = [c[0], c[1]] if sizes[c[0]]<sizes[c[1]] else [c[1],c[0]]
                break

        # generate the output
        if ids is not None:
            arrow_pt = np.array(self.list_add(boxes[ids[0]][0], self.l515_roi[:2]))
            arrow_pt_3d = read_point_from_region(xy_to_yx(arrow_pt), depth, region=3, camera_intrinsics=camera_intrinsics)
            arrow_pt_3d = self.transform_point(arrow_pt_3d, transform)

            # anchor_pt = self.list_add(boxes[ids[1]][0], self.l515_roi[:2])
            anchor_box = boxes[ids[1]]
            anchor_half_length = max(anchor_box[1])/2
            anchor_pt = np.array(anchor_box[0])
            anchor_pt = self.list_add(anchor_pt, self.l515_roi[:2])
            aa_distance = euclidian_distance(anchor_pt, arrow_pt)
            anchor_pt = anchor_pt+(anchor_pt-arrow_pt)*anchor_half_length/aa_distance/4*3
            anchor_pt_3d = read_point_from_region(xy_to_yx(anchor_pt), depth, region=3, camera_intrinsics=camera_intrinsics)
            anchor_pt_3d = self.transform_point(anchor_pt_3d, transform)

            yaw = atan2(arrow_pt_3d[1]-anchor_pt_3d[1],arrow_pt_3d[0]-anchor_pt_3d[0])
            pose = np.concatenate([anchor_pt_3d, quaternion_from_euler(0, 0, yaw)])

            img = self.generate_frame(img, [boxes[ids[0]], boxes[ids[1]], (tuple(anchor_pt-self.l515_roi[:2]), min(anchor_box[1]))])
            return pose, img
        else:
            return None, img

    def eyelet_detection(self, camera, seg):
        '''
        [Main recognition function to detect the eyelets]
        Input: image
        output: list of poses (n*7, n for number of eyelets), result image
        '''

        img = camera.read_image()
        depth = camera.read_depth()
        camera_intrinsics = camera.read_camera_info()
        # if True:
        #     pose = camera.read_camera_pose()
        #     (trans,quat) = pose[:3], pose[3:]
        # else:
        (trans,quat) = self.listener.lookupTransform(self.robot_frame, camera.frame, rospy.Time(0))
        transform = compose_matrix(angles=euler_from_quaternion(quat), translate=trans)
        
        mask = seg.predict_img(img)

        # cap with depth
        mask = mask*(depth<400) # remove everything beyond 0.5m
        
        if self.debug:
            self.mask_pub.publish(msgify(Image, mask, 'mono8'))
        
        h,w = mask.shape
        mask[int(350/480*h):, int(620/848*w):int(700/848*w)] = 0 # remove aglets

        # smooth the mask
        # mask = cv2.GaussianBlur(mask,(5,5),0)

        # hierarchy: [Next, Previous, First_Child, Parent]
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or hierarchy is None:
            return [], img, []

        hierarchy = hierarchy[0]
        # approximate the contours with circles
        circles = []
        outer_circles = []
        outer_contours = []
        inner_circle_boxes = []
        outer_circle_boxes = []
        for i in range(len(contours)):
            if hierarchy[i,3] == -1:
                continue
            inner_circle = cv2.minEnclosingCircle(contours[i]) # (x,y),radius inner_circle
            outer_circle = cv2.minEnclosingCircle(contours[hierarchy[i, 3]]) # (x,y),radius outer_circle
            if outer_circle[1] >= 30 or inner_circle[1] <= 5: # remove unreasonable circles
                continue
            circles.append(inner_circle)
            outer_circles.append(outer_circle)
            outer_contours.append(contours[hierarchy[i, 3]])
            # radius = inner_circle[1]*2
            radius = (outer_circle[1]/sqrt(2)+inner_circle[1])
            inner_circle_box = (inner_circle[0], (radius, radius), 0) #(center(x, y), (width, height), angle of rotation)
            outer_circle_box = (outer_circle[0], (outer_circle[1]/sqrt(2), outer_circle[1]/sqrt(2)), 0) #(center(x, y), (width, height), angle of rotation)
            outer_circle_boxes.append(outer_circle_box)
            inner_circle_boxes.append(inner_circle_box)
        n_circles = len(circles)
        n_obs = 5
            
        # get poses of the circles
        poses_observations = []
        circles_points_observations = []
        for _ in range(n_circles):
            circles_points_observations.append([])
        for i in range(n_obs):
            depth = camera.read_depth()
            poses = []
            for id in range(n_circles):
                inner_circle_box = cv2.boxPoints(inner_circle_boxes[id])
                inner_circle_box = np.int0(inner_circle_box)

                # get the centre of the boxes
                box_3d = read_points_from_region(xy_to_yx(inner_circle_box), depth, region=5, camera_intrinsics=camera_intrinsics)
                # position_cam = np.mean(box_3d, axis=0)
                box_3d = [self.transform_point(v, transform) for v in box_3d]
                position = np.mean(box_3d, axis=0)

                # get all points on the outer contour
                for p in outer_contours[id]:
                    p_3d = depth_pixel_to_metric_coordinate(xy_to_yx(p[0]), depth, camera_intrinsics)
                    if not isnan(p_3d[0]):
                        p_3d = self.transform_point(p_3d, transform)
                        circles_points_observations[id].append(p_3d)

                poses.append(np.concatenate((position, [0]*3)))

            poses_observations.append(poses)
        poses = np.mean(poses_observations, axis=0) # average out

        for id in range(n_circles):
            open3d_cloud = o3d.geometry.PointCloud()
            open3d_cloud.points = o3d.utility.Vector3dVector(circles_points_observations[id])
            plane_model, inliers = open3d_cloud.segment_plane(distance_threshold=0.005,
                                                    ransac_n=3,
                                                    num_iterations=1500)
            normal = plane_model[:3]/np.linalg.norm(plane_model[:3]) # renormalise
            if camera == self.d435_l:
                if normal[1]>0: normal = [ -v for v in normal] # force to face the shoe centre
            else:
                if normal[1]<0: normal = [ -v for v in normal] # force to face the shoe centre
            poses[id, 3:] = normal
        
        confidences = []
        for pose in poses:
            # normal = pose[3:]/np.linalg.norm(pose[3:]) # renormalise
            normal = pose[3:]
            # compute confidence
            ae_vec = np.subtract(pose[:3], trans)
            # if normal[1]>0: normal = [ -v for v in normal] # force to face the shoe centre
            c = np.dot(ae_vec,normal)/np.linalg.norm(ae_vec)/np.linalg.norm(normal) # -> cosine of the angle
            angle = np.arccos(np.clip(c, -1, 1)) # the angle
            confidence = 1-angle/pi
            confidences.append(confidence if not np.isnan(confidence) else 0)

        result_img = self.generate_frame(img, outer_circles+circles, list(confidences)*2)
        result_img = self.generate_frame(result_img, inner_circle_boxes, confidences)
        return poses, result_img, confidences

    @staticmethod
    def transform_point(point, transfer):
        return np.dot(transfer, np.array([point[0], point[1], point[2], 1.0]))[:3]

    @staticmethod
    def list_add(list_a,list_b):
        return [a+b for a,b in zip(list_a, list_b)]

    def generate_frame(self, img, shapes, confidence=None):
        '''
        [Generate the new frame for display]
        Input: image, shapes [(centre, radius) or (x, y, w, h) (numpy)]
        output: resulting image
        '''
        if confidence is None or np.any(np.isnan(confidence)):
            confidence = len(shapes)*[1]
        confidence = [max((c-0.7)/0.3, 0) for c in confidence]
        for id, shape in enumerate(shapes):
            color = int(255*confidence[id])
            if len(shape)==2: # if the shape is a circle
                img = cv2.circle(img, tuple(np.int0(shape[0])), int(shape[1]), (0,color,0), 2)
                # if id < len(shapes)//2:
                #     cv2.putText(img, str(id), tuple(np.int0(shape[0])), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,color,0), 2, 8)
            elif len(shape)==3: # if the shape is a box
                box = cv2.boxPoints(shape)
                box = np.int0(box)
                img = cv2.drawContours(img,[box],0,(0,0,color),2)            
        return img

if __name__ == "__main__":
    sl_visn_node = shoelacingVision()
