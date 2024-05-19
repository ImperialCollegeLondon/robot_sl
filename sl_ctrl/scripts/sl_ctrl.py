import threading
import numpy as np
from os import path
from scipy.spatial import distance
from math import cos, sin, pi, sqrt

from sl_params import ShoelacingParameters
from utils import list_to_pose_msg, ls_concat, ls_add, tf_ls2mat, tf_mat2ls, pose_msg_to_list, is_sorted

import tf
import rospy
import rospkg
from sl_msgs.srv import findTargetsService, findTargetsServiceRequest
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion, compose_matrix

# rospy.loginfo("..............................T E S T..................................")


class ShoelacingPrimitives:
    package_name = 'sl_ctrl'
    marker_topic = '/visualization_marker'
    # robot_frame = "yumi_base_link"
    vision_srv = 'find_targets'
    log_topic = 'sl_logs'
    aglet_owner_topic = 'aglet_owner'
    aglet_pose_topic = 'aglet_pose'
    hand_pose_topic = 'hand_pose'
    eyelet_pose_topic = 'eyelet_pose'
    cursor_topic = 'cursor'

    def __init__(self, auto_execution, reset=True, start_id=0, sim=False):
        # ros related initialisation
        self.logs_pub = rospy.Publisher(self.log_topic, String, queue_size=1)
        self.aglet_owner_pub = rospy.Publisher(self.aglet_owner_topic, Int32MultiArray, queue_size=1)
        self.aglet_pose_pub = rospy.Publisher(self.aglet_pose_topic, PoseArray, queue_size=1)
        self.eyelet_pose_pub = rospy.Publisher(self.eyelet_pose_topic, PoseArray, queue_size=1)
        self.hand_pose_pub = rospy.Publisher(self.hand_pose_topic, PoseArray, queue_size=1)
        self.cursor_pub = rospy.Publisher(self.cursor_topic, Int32MultiArray, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.find_targets = rospy.ServiceProxy(self.vision_srv, findTargetsService)
        rospy.wait_for_service(self.vision_srv)

        # load params
        self.sim = sim
        self.debug = False
        self.pkg_path = rospkg.RosPack().get_path(self.package_name)
        self.pm = ShoelacingParameters(reset=True, start_id=start_id,
                                           config_path=path.join(self.pkg_path, 'params' if not self.sim else 'params/unity'),
                                           result_path=path.join(self.pkg_path, 'results'),
                                           log_handle=self.add_to_log)
        self.update_cursor('left', self.pm.left_cursor)
        self.update_cursor('right', self.pm.right_cursor)

        # initialise yumi wrapper        
        if self.sim:
            from sl_yumi_wrapper_unity import YuMiWrapper
        else:
            from sl_yumi_wrapper import YuMiWrapper
        self.yumi = YuMiWrapper(auto_execution,
            workspace=self.pm.workspace,
            vel_scale=self.pm.vel_scale,
            gp_opening=self.pm.aglet_thickness*1000*2, # to mm
            table_offset=self.pm.table_offset,
            grasp_states=self.pm.grasp_states,
            grasp_states2=self.pm.grasp_states2,
            observe_states=self.pm.observe_states)

        # scan the shoe
        self.pm.update_yumi_constriants = self.yumi.update_sl_constriants
        self.init_eyelet_poses()
        
        rospy.loginfo('Execution module ready.')

        # create and start the tf listener thread
        self.side_queue = []
        self.side_thread = threading.Thread(target=self.tf_thread_func, daemon=True)
        self.side_thread.start()

    def tf_thread_func(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                if 'left_gripper' in self.pm.aglet_at.values():
                    (trans,rot) = self.tf_listener.lookupTransform(self.yumi.robot_frame, self.yumi.ee_frame_l, rospy.Time(0))
                    self.pub_hand_poses('left_gripper', trans+rot)
                    pose = tf_mat2ls(tf_ls2mat(trans+rot)@self.yumi.gripper_l_to_aglet)
                    aglet = self.get_aglet_at('left_gripper')
                    self.pub_aglet_pose(aglet, pose)
                elif 'right_gripper' in self.pm.aglet_at.values():
                    (trans,rot) = self.tf_listener.lookupTransform(self.yumi.robot_frame, self.yumi.ee_frame_r, rospy.Time(0))
                    self.pub_hand_poses('right_gripper', trans+rot)
                    pose = tf_mat2ls(tf_ls2mat(trans+rot)@self.yumi.gripper_r_to_aglet)
                    aglet = self.get_aglet_at('right_gripper')
                    self.pub_aglet_pose(aglet, pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

    def execute(self, action, next_action, sl_cost):
        '''
        pick: gripper aglet site
        ''' 
        self.add_to_log(f'{action} Cost: {sl_cost}')
        # interpret the action
        if 'left_insert' in action[0]:
            reset = False if next_action is not None and next_action[0][:4]=='left' else True
            self.left_lace('eyelets_b', int(action[1][6:]), 'aglet{}'.format(action[0][-2:]), 
                                     sl_cost=sl_cost+self.pm.eyelet_to_edge, reset=reset, site=action[-1])
        elif 'right_insert' in action[0]:
            reset = False if next_action is not None and next_action[0][:5]=='right' else True
            self.right_lace('eyelets_r', int(action[1][6:]), 'aglet{}'.format(action[0][-2:]), 
                                       sl_cost=sl_cost+self.pm.eyelet_to_edge, reset=reset, site=action[-1])
        elif action[0] == 'right_to_left_transfer':
            reset = False if next_action is not None and next_action[0][:4]=='left' else True
            self.right_to_left(action[1], reset=reset, site=action[-1])
        elif action[0] == 'left_to_right_transfer':
            reset = False if next_action is not None and next_action[0][:5]=='right' else True
            self.left_to_right(action[1], reset=reset, site=action[-1])
        elif action[0] == 'left_replace':
            self.left_replace(action[1], site=action[-1])
        elif action[0] == 'right_replace':
            self.right_replace(action[1], site=action[-1])
        else:
            print('Unrecogised primitive name ({})!'.format(action[0]))

    def right_pick(self, aglet, fine_ori=True):
        """ pick up the aglet with the right gripper """
        # calc pick poses
        target, yaw = self.get_aglet_poses(aglet)
        pick_pos = [target[0], target[1], target[2]+self.pm.gp_os]
        pick_pos_approach = ls_add(pick_pos, [0, 0, self.pm.app_os])
        pick_rot = self.pm.grasp_rot_r
        pick_rot_fine = ls_add(pick_rot, [0, 0, yaw])

        self.yumi.remove_table()
        # approach the aglet
        waypoints = []
        waypoints.append(ls_concat(pick_pos_approach,pick_rot))
        self.yumi.right_go_thro(waypoints,"Pick Approach")
        self.yumi.open_right_gripper()

        # pick the aglet
        waypoints = []
        waypoints.append(ls_concat(pick_pos,pick_rot_fine))
        self.yumi.right_go_thro(waypoints,"Pick")
        self.yumi.close_right_gripper()

        self.update_aglet_ownership(aglet, 'right_gripper')

        # retreat after pick
        waypoints = []
        waypoints.append(ls_concat(pick_pos_approach,pick_rot_fine))
        self.yumi.right_go_thro(waypoints,"Pick Retract")
        self.yumi.add_table()

        # set section flags
        self.pm.update_section_availability(aglet, None)

        if fine_ori:
            self.right_refine_orientation()

    def left_pick(self, aglet, fine_ori=True):
        """ pick up the aglet with the left gripper """
        # calc pick poses
        target, yaw = self.get_aglet_poses(aglet)
        pick_pos = [target[0], target[1], target[2]+self.pm.gp_os]
        pick_pos_approach = ls_add(pick_pos, [0, 0, self.pm.app_os])
        pick_rot = self.pm.grasp_rot_l
        pick_rot_fine = ls_add(pick_rot, [0, 0, yaw])
        
        self.yumi.remove_table()
        # approach the aglet
        waypoints = []
        waypoints.append(ls_concat(pick_pos_approach,pick_rot))
        self.yumi.left_go_thro(waypoints,"Pick Approach")
        self.yumi.open_left_gripper()

        # pick the aglet
        waypoints = []
        waypoints.append(ls_concat(pick_pos,pick_rot_fine))
        self.yumi.left_go_thro(waypoints,"Pick")
        self.yumi.close_left_gripper()
        self.update_aglet_ownership(aglet, 'left_gripper')

        # retreat after pick
        waypoints = []
        waypoints.append(ls_concat(pick_pos_approach,pick_rot_fine))
        self.yumi.left_go_thro(waypoints,"Pick Retract")

        self.yumi.add_table()

        # set section flags
        self.pm.update_section_availability(aglet, None)

        if fine_ori:
            self.left_refine_orientation()

    def right_place(self, aglet, site='site_r1'):
        """ place down the aglet with the right gripper"""

        # stretch the shoelace
        self.add_to_log("Placing to "+site)
        if site == 'site_r1':
            self.yumi.right_go_grasp()
            section = self.pm.site_r1
            self.right_stretch_backward(aglet)
            self.yumi.right_go_grasp()
        elif site == 'site_r2':
            section = self.pm.site_r2
            self.right_stretch_backward(aglet)
            self.yumi.right_go_grasp()
        elif site == 'site_r3':
            section = self.pm.site_r3
            self.right_stretch_forward(aglet)
            self.yumi.right_go_grasp2()
        else:
            rospy.logerr("No Section is available for placing!")
            return

        self.pm.update_section_availability(aglet, site)

        # calc place poses
        place_pos = [section[0], section[1], section[2]+self.pm.gp_os]
        place_pos_approach = ls_add(place_pos, [0, -self.pm.app_os, self.pm.app_os])
        place_rot = self.pm.grasp_rot_r
        self.yumi.remove_table()
        # approach the aglet
        waypoints = []
        waypoints.append(place_pos_approach+place_rot)
        self.yumi.right_go_thro(waypoints,"Place Approach", eef_step=0.05)
        # place the aglet
        waypoints = []
        waypoints.append(place_pos+place_rot)
        self.yumi.right_go_thro(waypoints,"Place")
        self.yumi.open_right_gripper(full=True)
        self.update_aglet_ownership(aglet, site)
        # retreat after place
        waypoints = []
        waypoints.append(place_pos_approach+place_rot)
        self.yumi.right_go_thro(waypoints,"Place Retract")
        self.yumi.add_table()

    def left_place(self, aglet, site='site_l1'):
        """ place down the aglet with the right gripper"""

        # stretch the shoelace
        self.add_to_log("Placing to "+site)
        if site == 'site_l1':
            self.yumi.left_go_grasp()
            section = self.pm.site_l1
            self.left_stretch_backward(aglet)
            self.yumi.left_go_grasp()
        elif site == 'site_l2':
            section = self.pm.site_l2
            self.left_stretch_backward(aglet)
            self.yumi.left_go_grasp()
        elif site == 'site_l3':
            section = self.pm.site_l3
            self.left_stretch_forward(aglet)
            self.yumi.left_go_grasp2()
        else:
            rospy.logerr("No Section is available for placing!")
            return

        self.pm.update_section_availability(aglet, site)

        # calc place poses
        place_pos = [section[0], section[1], section[2]+self.pm.gp_os]
        place_pos_approach = ls_add(place_pos, [0, self.pm.app_os, self.pm.app_os])
        place_rot = self.pm.grasp_rot_l
        self.yumi.remove_table()
        # approach the aglet
        waypoints = []
        waypoints.append(place_pos_approach+place_rot)
        self.yumi.left_go_thro(waypoints,"Place Approach", eef_step=0.05)
        # place the aglet
        waypoints = []
        waypoints.append(place_pos+place_rot)
        self.yumi.left_go_thro(waypoints,"Place")
        self.yumi.open_left_gripper(full=True)
        self.update_aglet_ownership(aglet, site)
        # retreat after place
        waypoints = []
        waypoints.append(place_pos_approach+place_rot)
        self.yumi.left_go_thro(waypoints,"Place Retract")
        self.yumi.add_table()

    def right_refine_orientation(self):
        # calc transfer poses
        transfer_point_l = ls_add(self.pm.hand_over_centre_2, 
                                  [self.pm.da_os_x,self.pm.aglet_length/2,0])
        pinch_rot_l = [-pi/2, 0, 0]
        pinch_rot_l2 = [-pi/2, -pi/2, 0]
        pinch_rot_r = [0, 0, pi/2]
        transfer_point_l_retreat = ls_add(transfer_point_l, [0, self.pm.app_os*2, 0])
        transfer_point_r_retreat = ls_add(self.pm.hand_over_centre_2, [0, 0, self.pm.app_os])

        # reset left arm
        self.yumi.left_go_observe(main=False)
        self.yumi.open_left_gripper(main=False, full=1)
        # reset right arm
        self.yumi.right_go_transfer()
        self.yumi.wait_for_side_thread()
        # pinch the aglet
        waypoints = [ls_concat(transfer_point_l, pinch_rot_l)]
        self.yumi.left_tip_go_thro(waypoints, "Reorient Pinch 1")
        self.yumi.close_left_gripper(full_force=False)
        self.yumi.open_left_gripper()
        # pinch the aglet again
        waypoints = [ls_concat(transfer_point_l, pinch_rot_l2)]
        self.yumi.left_tip_go_thro(waypoints, "Reorient Pinch 2")
        self.yumi.close_left_gripper(full_force=False)
        self.yumi.open_left_gripper()
        # retreat left arm
        waypoints = [ls_concat(transfer_point_l_retreat, pinch_rot_l2)]
        self.yumi.left_tip_go_thro(waypoints, "Reorient Left Retract")
        self.yumi.left_go_observe(main=False)
        self.yumi.close_left_gripper(full_force=False)
        # retreat after action
        waypoints = [ls_concat(transfer_point_r_retreat, pinch_rot_r)]
        self.yumi.right_tip_go_thro(waypoints, "Reorient Right Retract")
        self.yumi.wait_for_side_thread()

    def left_refine_orientation(self):
        #### change orientation
        transfer_point_r = ls_add(self.pm.hand_over_centre_2, 
                                  [-self.pm.da_os_x,-self.pm.aglet_length/2,-self.pm.da_os_z])
        pinch_rot_r = [pi/2, 0, 0]
        pinch_rot_r2 = [pi/2, -pi/2, 0]
        pinch_rot_l = [0, 0, -pi/2]
        transfer_point_r_retreat = ls_add(transfer_point_r, [0, -self.pm.app_os*2, 0])
        transfer_point_l_retreat = ls_add(self.pm.hand_over_centre_2, [0, 0, self.pm.app_os])

        # reset right arm
        self.yumi.right_go_observe(main=False)
        self.yumi.open_right_gripper(main=False, full=1)
        # reset left arm
        self.yumi.left_go_transfer()        
        self.yumi.wait_for_side_thread()
        # pinch the aglet
        waypoints = [ls_concat(transfer_point_r, pinch_rot_r)]
        self.yumi.right_tip_go_thro(waypoints, "Reorient Pinch 1")
        self.yumi.close_right_gripper(full_force=False)
        self.yumi.open_right_gripper()
        # pinch the aglet again
        waypoints = [ls_concat(transfer_point_r, pinch_rot_r2)]
        self.yumi.right_tip_go_thro(waypoints, "Reorient Pinch 2")
        self.yumi.close_right_gripper(full_force=False)
        self.yumi.open_right_gripper()
        # retreat left arm
        waypoints = [ls_concat(transfer_point_r_retreat, pinch_rot_r2)]
        self.yumi.right_tip_go_thro(waypoints, "Reorient Right Retract")
        self.yumi.right_go_observe(main=False)
        self.yumi.close_right_gripper(full_force=False)
        # retreat after action
        waypoints = [ls_concat(transfer_point_l_retreat, pinch_rot_l)]
        self.yumi.left_tip_go_thro(waypoints, "Reorient Left Retract")
        self.yumi.wait_for_side_thread()

    def right_stretch_backward(self, aglet):
        self.yumi.change_speed(0.25)
        sl_length = self.pm.get_shoelace_length(aglet)
        root = self.pm.get_root_position(aglet)
        # calc stretch poses
        stretch_z = 0.2
        stretch_y = -sqrt(sl_length**2-(stretch_z-self.pm.gp_os-root[2])**2)*cos(pi/4)+root[1]
        stretch_y = max(stretch_y, -0.3)
        stretch_x = -sqrt(sl_length**2-(stretch_y-root[1])**2-(stretch_z-self.pm.gp_os-root[2])**2)
        stretch_point = [root[0]+stretch_x, stretch_y, stretch_z]
        stretch_point_retract = [root[0]+stretch_x/2, stretch_y, stretch_z]
        stretch_rot = [0, pi, 0]
        self.add_to_log("[Right stretch backward to] "+str(stretch_point))

        # stretch the shoelace backward
        waypoints = []
        waypoints.append(ls_concat(stretch_point, stretch_rot))
        self.yumi.right_go_thro(waypoints,"Stretch Backward") 
        rospy.sleep(1)
        waypoints = []
        waypoints.append(ls_concat(stretch_point_retract, stretch_rot))
        self.yumi.right_go_thro(waypoints,"Stretch Backward Retract")
        self.yumi.change_speed(1)

    def right_stretch_forward(self, aglet):
        self.yumi.change_speed(0.5)
        sl_length = self.pm.get_shoelace_length(aglet)
        root = self.pm.get_root_position(aglet)
        # calc stretch poses
        stretch_x = max(root[0], self.pm.site_r1[0]+self.pm.app_os)-self.pm.gp_os
        stretch_z = 0.25
        stretch_y = root[1]-sqrt(sl_length**2-(stretch_z-root[2])**2-(stretch_x+self.pm.gp_os-root[0])**2)
        stretch_point = [stretch_x, stretch_y, stretch_z]
        stretch_point_retract = [stretch_x, stretch_y/2, stretch_z]
        stretch_rot = [0, pi/2, 0]
        self.add_to_log("[Right stretch forward to] "+str(stretch_point))

        # stretch the shoelace forward
        waypoints = []
        waypoints.append(ls_concat(stretch_point, stretch_rot))
        self.yumi.right_go_thro(waypoints,"Stretch Forward")
        rospy.sleep(1)
        waypoints = []
        waypoints.append(ls_concat(stretch_point_retract, stretch_rot))
        self.yumi.right_go_thro(waypoints,"Stretch Forward Retract")
        self.yumi.change_speed(1)

    def left_stretch_backward(self, aglet):
        self.yumi.change_speed(0.25)
        sl_length = self.pm.get_shoelace_length(aglet)
        root = self.pm.get_root_position(aglet)
        # calc stretch poses
        stretch_z = 0.2
        stretch_y = sqrt(sl_length**2-(stretch_z-self.pm.gp_os-root[2])**2)*cos(pi/4)+root[1]
        stretch_y = min(stretch_y, 0.3)
        stretch_x = -sqrt(sl_length**2-(stretch_y-root[1])**2-(stretch_z-self.pm.gp_os-root[2])**2)
        stretch_point = [root[0]+stretch_x, stretch_y, stretch_z]
        stretch_point_retract = [root[0]+stretch_x/2, stretch_y, stretch_z]
        stretch_rot = [0, pi, 0]
        self.add_to_log("[Left stretch backward to] "+str(stretch_point))

        # stretch the shoelace backward
        waypoints = []
        waypoints.append(ls_concat(stretch_point, stretch_rot))
        self.yumi.left_go_thro(waypoints,"Stretch Backward")
        rospy.sleep(1)
        waypoints = []
        waypoints.append(ls_concat(stretch_point_retract, stretch_rot))
        self.yumi.left_go_thro(waypoints,"Stretch Backward Retract")
        self.yumi.change_speed(1)
    
    def left_stretch_forward(self, aglet):
        self.yumi.change_speed(0.5)
        sl_length = self.pm.get_shoelace_length(aglet)
        root = self.pm.get_root_position(aglet)
        # calc stretch poses
        stretch_x = max(root[0], self.pm.site_l1[0]+self.pm.app_os)-self.pm.gp_os
        stretch_z = 0.25
        stretch_y = root[1]+sqrt(sl_length**2-(stretch_z-root[2])**2-(stretch_x+self.pm.gp_os-root[0])**2)
        stretch_point = [stretch_x, stretch_y, stretch_z]
        stretch_point_retract = [stretch_x, stretch_y/2, stretch_z]
        stretch_rot = [0, pi/2, 0]
        self.add_to_log("[Left stretch forward to] "+str([stretch_x, stretch_y, stretch_z]))

        # stretch the shoelace forward
        waypoints = []
        waypoints.append(ls_concat(stretch_point, stretch_rot))
        self.yumi.left_go_thro(waypoints,"Stretch Forward")
        rospy.sleep(1)
        waypoints = []
        waypoints.append(ls_concat(stretch_point_retract, stretch_rot))
        self.yumi.left_go_thro(waypoints,"Stretch Forward Retract")
        self.yumi.change_speed(1)

    def right_lace(self, eyelet_group, eyelet_id, aglet, sl_cost=0, reset=True, site='site_r1'):
        """
        This primitive laces eyelets on the right eyestay, inserts aglet from the right side
        Process: right pick, left to grasp, right insert, right retract, left retract, left place
        """

        ''' PICKING '''
        self.right_pick(aglet)

        ''' locate the eyelet '''
        # make observations
        self.get_eyelet_poses(eyelet_group, target_id=eyelet_id//2, fine=False, 
                              compare_with=self.pm.eyelet_poses[self.pm.right_cursor])
        e_pos_relax = self.pm.eyelet_poses[eyelet_id][:3]

        ''' prepare to grasp '''
        # calc grasp poses
        grasp_pitch = self.pm.insert_pitch2
        grasp_rot = self.yumi.ee_rot_to_tip_l([0, pi/2+grasp_pitch, -pi])
        grasp_pos = ls_add(e_pos_relax, [-self.pm.eyelet_radius*cos(grasp_pitch),
                                         self.pm.eyestay_thickness,
                                         -self.pm.eyelet_radius*sin(grasp_pitch)])
        grasp_pos_approach = ls_add(e_pos_relax, [self.pm.eyelet_to_edge*cos(grasp_pitch),
                                                  self.pm.eyestay_opening/2+self.pm.eyestay_thickness,
                                                  self.pm.eyelet_to_edge*sin(grasp_pitch)])
        grasp_pos_approach2 = ls_add(grasp_pos, [0, self.pm.eyestay_opening/2, 0])

        # prepare to grasp
        self.yumi.left_go_thro([ls_concat(self.pm.pre_grasp, self.pm.grasp_rot_r)], "Lace Grasp Prepare")
        self.yumi.wait_for_side_thread()
        self.yumi.left_tip_go_thro([ls_concat(grasp_pos_approach, grasp_rot)], "Lace Grasp Approach")
        self.yumi.open_left_gripper(full=2)
        self.yumi.change_speed(0.5)
        waypoints = [ls_concat(grasp_pos_approach2, grasp_rot),
                     ls_concat(grasp_pos, grasp_rot)]
        self.yumi.left_tip_go_thro(waypoints, "Lace Grasp")

        ''' locate the eyelet '''
        # correct the previous estimation again
        self.get_eyelet_poses(eyelet_group, target_id=eyelet_id//2, init=False, fine=True,
                              compare_with=e_pos_relax)
        eyelet_pos = self.pm.eyelet_poses[eyelet_id][:3]
            
        ''' insert the aglet '''
        # reset right arm
        self.yumi.right_go_grasp2()
        # calc insert poses
        insert_pitch = grasp_pitch-pi/8 # minus gripper cam mount angle
        e_pose = compose_matrix(translate=eyelet_pos, angles=[0, 0, pi/2]) # eyelet rotation too noisy
        insert_pose = tf_mat2ls(e_pose@tf_ls2mat([-self.pm.eyestay_thickness/2,0,0,
                                                  -insert_pitch, 0, 0]))
        insert_pose_approach = tf_mat2ls(e_pose@tf_ls2mat([-self.pm.app_os*2,0,0,
                                                           -insert_pitch, 0, 0]))
        retract_pos_rel = [0, self.pm.app_os*(cos(insert_pitch)+1), self.pm.app_os*sin(insert_pitch)]
        insert_pose_retract = tf_mat2ls(e_pose@tf_ls2mat(retract_pos_rel+[-insert_pitch, 0, 0]))
        insert_pose_retract2 = tf_mat2ls(e_pose@tf_ls2mat(ls_add(retract_pos_rel, [-self.pm.app_os*2, 0, 0])+
                                                          [-insert_pitch, 0, 0]))
        # adjust the gripper angle for the insertion
        self.yumi.right_go_thro([self.pm.pre_insert_r+[0, pi-insert_pitch, 0]], "Lace Insert Prepare")
        # insert with the right gripper
        self.yumi.right_tip_go_thro([insert_pose_approach, insert_pose], "Lace Insert", velocity_scaling=1.5)
        self.yumi.close_left_gripper()
        self.yumi.open_right_gripper()
        self.update_aglet_ownership(aglet, 'left_gripper')
        # retract the left gripper after the insertion
        self.yumi.right_tip_go_thro([insert_pose_retract], "Lace Insert Retract")
        self.yumi.open_right_gripper(full=True)

        ''' grasp and pull '''
        # calc grasp retract poses
        retract_pos = ls_add(e_pos_relax, [-self.pm.eyelet_radius*cos(grasp_pitch),
                                            self.pm.aglet_length+self.pm.gp_tip_w,
                                           -self.pm.eyelet_radius*sin(grasp_pitch)])
        retract_pos2 = ls_add(retract_pos, [self.pm.eyelet_radius, 0, 0])
        retract_pos3 = ls_add(retract_pos, [self.pm.eyelet_radius, 0, self.pm.app_os*2])
        retract_pos4 = ls_add(self.pm.hand_over_centre, [0, self.pm.app_os*2, 0])
        stretch_rot = [0,0,pi]
        ## pull out of the eyelet with the left gripper
        waypoints = [ls_concat(retract_pos, grasp_rot),
                     ls_concat(retract_pos2, grasp_rot)]
        self.yumi.left_tip_go_thro(waypoints, "Lace Grasp Retract")
        waypoints = [ls_concat(retract_pos3, grasp_rot),
                     ls_concat(retract_pos4, stretch_rot)]
        self.yumi.left_tip_go_thro(waypoints, "Lace Grasp Retract 2")
        self.yumi.right_tip_go_thro([insert_pose_retract2], "Lace Insert Retract 2", main=False)
        self.yumi.close_right_gripper(main=False)

        # reset left arm
        self.yumi.left_go_grasp2()
        self.yumi.change_speed(1)
        # update parameters
        self.pm.update_shoelace_length(aglet, -sl_cost)
        self.pm.update_root_position(aglet, e_pos_relax)
        self.update_cursor('right', self.pm.right_cursor+2)

        ''' PLACING '''
        # place the aglet
        self.left_place(aglet, site=site)

        # reset arms
        self.yumi.close_right_gripper(main=False)
        if reset:
            self.yumi.right_go_observe()
        self.yumi.wait_for_side_thread()

    def left_lace(self, eyelet_group, eyelet_id, aglet, sl_cost=0, reset=True, site='site_l1'):
        """
        This primitive laces eyelets on the left eyestay, inserts aglet from the left side
        Process: left pick, right to grasp, left insert, left retract, right retract, right place
        """

        ''' pick the aglet '''
        self.left_pick(aglet)

        ''' locate the eyelet '''
        # make observations
        self.get_eyelet_poses(eyelet_group, target_id=eyelet_id//2, fine=False, 
                              compare_with=self.pm.eyelet_poses[self.pm.left_cursor])
        e_pos_relax = self.pm.eyelet_poses[eyelet_id][:3]

        ''' prepare to grasp '''
        # calc grasp poses
        grasp_pitch = self.pm.insert_pitch2
        grasp_rot = self.yumi.ee_rot_to_tip_r([0, pi/2+grasp_pitch, pi])
        grasp_pos = ls_add(e_pos_relax, [-self.pm.eyelet_radius*cos(grasp_pitch), 
                                         -self.pm.eyestay_thickness, 
                                         -self.pm.eyelet_radius*sin(grasp_pitch)])
        grasp_pos_approach = ls_add(e_pos_relax, [self.pm.eyelet_to_edge*cos(grasp_pitch),
                                                  -self.pm.eyestay_opening/2-self.pm.eyestay_thickness,
                                                  self.pm.eyelet_to_edge*sin(grasp_pitch)])
        grasp_pos_approach2 = ls_add(grasp_pos, [0, -self.pm.eyestay_opening/2, 0])

        # prepare to grasp
        self.yumi.right_go_thro([ls_concat(self.pm.pre_grasp, self.pm.grasp_rot_l)], "Lace Grasp Prepare")
        self.yumi.wait_for_side_thread()
        self.yumi.right_tip_go_thro([ls_concat(grasp_pos_approach, grasp_rot)], "Lace Grasp Approach")
        self.yumi.open_right_gripper(full=2) # mode 2: half open
        self.yumi.change_speed(0.5)
        waypoints=[ls_concat(grasp_pos_approach2, grasp_rot),
                   ls_concat(grasp_pos, grasp_rot)]
        self.yumi.right_tip_go_thro(waypoints, "Lace Grasp")

        ''' locate the eyelet '''
        # correct the previous estimation again
        self.get_eyelet_poses(eyelet_group, target_id=eyelet_id//2, init=False, fine=True, 
                              compare_with=e_pos_relax)
        eyelet_pos = self.pm.eyelet_poses[eyelet_id][:3]

        ''' insert the aglet '''
        # reset left arm
        self.yumi.left_go_observe()
        self.yumi.left_go_grasp2()
        # calc insert poses
        insert_pitch = grasp_pitch-pi/8 # minus gripper cam mount angle
        e_pose = compose_matrix(translate=eyelet_pos, angles=[0, 0, -pi/2]) # eyelet rotation too noisy
        insert_pose = tf_mat2ls(e_pose@tf_ls2mat([-self.pm.eyestay_thickness/2,0,0, 
                                                            insert_pitch, 0, 0]))
        insert_pose_approach = tf_mat2ls(e_pose@tf_ls2mat([-self.pm.app_os*2,0,0, 
                                                           insert_pitch, 0, 0]))
        retract_pos_rel = [0, -self.pm.app_os*(cos(insert_pitch)+1), self.pm.app_os*sin(insert_pitch)]
        insert_pose_retract = tf_mat2ls(e_pose@tf_ls2mat(retract_pos_rel+[insert_pitch, 0, 0]))
        insert_pose_retract2 = tf_mat2ls(e_pose@tf_ls2mat(ls_add(retract_pos_rel,[-self.pm.app_os*2,0,0])+
                                                          [insert_pitch, 0, 0]))
        # adjust the gripper angle for the insertion
        self.yumi.left_go_thro([ls_concat(self.pm.pre_insert_l, [0, pi-insert_pitch, 0])], "Lace Insert Prepare")

        # insert with the left gripper
        self.yumi.left_tip_go_thro([insert_pose_approach, insert_pose], "Lace Insert", velocity_scaling=1.5)
        self.yumi.close_right_gripper()
        self.yumi.open_left_gripper()
        self.update_aglet_ownership(aglet, 'right_gripper')
        # retract the left gripper after the insertion
        self.yumi.left_tip_go_thro([insert_pose_retract], "Lace Insert Retract")
        self.yumi.open_left_gripper(full=True)

        ''' grasp and pull '''
        # calc grasp retract poses
        retract_pos = ls_add(e_pos_relax, [-self.pm.eyelet_radius*cos(grasp_pitch),
                                           -self.pm.aglet_length-self.pm.gp_tip_w,
                                           -self.pm.eyelet_radius*sin(grasp_pitch)])
        retract_pos2 = ls_add(retract_pos, [self.pm.eyelet_radius, 0, 0])
        retract_pos3 = ls_add(retract_pos, [self.pm.eyelet_radius, 0, self.pm.app_os*2])
        retract_pos4 = ls_add(self.pm.hand_over_centre, [0, -self.pm.app_os*2, 0])
        stretch_rot = [0,0,pi]
        ## pull out of the eyelet with the right gripper
        waypoints = [ls_concat(retract_pos, grasp_rot),
                     ls_concat(retract_pos2, grasp_rot)]
        self.yumi.right_tip_go_thro(waypoints, "Lace Grasp Retract")
        waypoints=[ls_concat(retract_pos3,grasp_rot),
                   ls_concat(retract_pos4,stretch_rot)]
        self.yumi.right_tip_go_thro(waypoints, "Lace Grasp Retract 2")
        self.yumi.left_tip_go_thro([insert_pose_retract2], "Lace Insert Retract 2", main=False)
        self.yumi.close_left_gripper(main=False)

        # reset right arm
        self.yumi.right_go_grasp2()
        self.yumi.change_speed(1)
        # update parameters
        self.pm.update_shoelace_length(aglet, -sl_cost)
        self.pm.update_root_position(aglet, e_pos_relax)
        self.update_cursor('left', self.pm.left_cursor+2)

        ''' place the aglet '''
        # place the aglet
        self.right_place(aglet, site=site)

        # reset arms
        self.yumi.close_left_gripper(main=False)
        if reset:
            self.yumi.left_go_observe()
        self.yumi.wait_for_side_thread()

    def right_to_left_handover(self):
        # calc the transfer points
        centre = self.pm.hand_over_centre_2
        transfer_point_r = ls_add(centre, [0, -self.pm.gp_os, 0])
        transfer_point_r_approach = ls_add(transfer_point_r, [0, -self.pm.app_os, 0])
        transfer_rot_r = [-pi/2, pi, 0]
        transfer_point_l = ls_add(centre, [self.pm.da_os_x, self.pm.gp_os, self.pm.gp_tip_w])
        transfer_point_l_approach = ls_add(transfer_point_l, [0, self.pm.app_os, 0])
        transfer_rot_l = [pi/2, pi, 0]
        
        # right to preparation position
        waypoints = []
        waypoints.append(ls_concat(transfer_point_r_approach, transfer_rot_r))
        waypoints.append(ls_concat(transfer_point_r, transfer_rot_r))
        self.yumi.right_go_thro(waypoints, "Handover Right", main=False)
        # reset left arm
        self.yumi.left_go_observe()
        self.yumi.open_left_gripper()
        # grasp with left gripper
        waypoints = []
        waypoints.append(ls_concat(transfer_point_l_approach, transfer_rot_l))
        waypoints.append(ls_concat(transfer_point_l, transfer_rot_l))
        self.yumi.left_go_thro(waypoints, "Handover Left")
        self.yumi.wait_for_side_thread()
        self.yumi.close_left_gripper()
        self.yumi.open_right_gripper()
        self.update_aglet_ownership(self.get_aglet_at('right_gripper'), 'left_gripper')
        # retract right arm
        waypoints = []
        waypoints.append(ls_concat(transfer_point_r_approach, transfer_rot_r))
        self.yumi.right_go_thro(waypoints, "Handover Right Retract", main=True)
        # reset right arm
        self.yumi.close_right_gripper(main=False)
        self.yumi.right_go_observe(main=False)

    def left_to_right_handover(self):
        # calc the transfer points
        centre = self.pm.hand_over_centre_2
        transfer_point_l = ls_add(centre, [0, self.pm.gp_os, 0])
        transfer_point_l_approach = ls_add(transfer_point_l, [0, self.pm.app_os, 0])
        transfer_rot_l = [pi/2, pi, 0]
        transfer_point_r = ls_add(centre, [-self.pm.da_os_x, -self.pm.gp_os, self.pm.gp_tip_w])
        transfer_point_r_approach = ls_add(transfer_point_r, [0, -self.pm.app_os, 0])
        transfer_rot_r = [-pi/2, pi, 0]

        # reset left arm
        self.yumi.left_go_observe(main=False)
        # left to preparation position
        waypoints = []
        waypoints.append(ls_concat(transfer_point_l_approach, transfer_rot_l))
        waypoints.append(ls_concat(transfer_point_l, transfer_rot_l))
        self.yumi.left_go_thro(waypoints, "Handover Left", main=False)
        # reset right arm
        self.yumi.right_go_observe()
        self.yumi.open_right_gripper()
        # grasp with right gripper
        waypoints = []
        waypoints.append(ls_concat(transfer_point_r_approach, transfer_rot_r))
        waypoints.append(ls_concat(transfer_point_r, transfer_rot_r))
        self.yumi.right_go_thro(waypoints, "Handover Right")
        self.yumi.wait_for_side_thread()
        self.yumi.close_right_gripper()
        self.yumi.open_left_gripper()
        self.update_aglet_ownership(self.get_aglet_at('left_gripper'), 'right_gripper')
        # retract left arm
        waypoints = []
        # waypoints.append([centre[0]+extra_os_x, centre[1]+self.pm.gp_os+0.20, centre[2], pi/2, pi, 0])
        waypoints.append(ls_concat(transfer_point_l_approach, transfer_rot_l))
        self.yumi.left_go_thro(waypoints, "Handover Left Retract", main=False)
        # reset left arm
        self.yumi.close_left_gripper(main=False)
        self.yumi.left_go_observe(main=False)

    def right_replace(self, aglet, site='site_r1'):
        self.right_pick(aglet, fine_ori=False)
        self.right_place(aglet, site=site)

    def left_replace(self, aglet, site='site_l1'):
        self.left_pick(aglet, fine_ori=False)
        self.left_place(aglet, site=site)

    def right_to_left(self, aglet, reset=True, site='site_l1'):
        self.right_pick(aglet, fine_ori=False)
        self.right_to_left_handover()
        self.left_place(aglet, site=site)

        # reset to initial position
        self.yumi.close_left_gripper(main=False)
        if reset:
            self.yumi.left_go_observe()
        self.yumi.wait_for_side_thread()

    def left_to_right(self, aglet, reset=True, site='site_r1'):
        self.left_pick(aglet, fine_ori=False)
        self.left_to_right_handover()
        self.right_place(aglet, site=site)

        # reset to initial position
        self.yumi.close_right_gripper(main=False)
        if reset:
            self.yumi.right_go_observe()
        self.yumi.wait_for_side_thread()

    def call_vision_srv(self, target_name, camera_name):
        '''
        input: target_name, camera_name
        output: target_poses, confidence
        '''
        request = findTargetsServiceRequest()
        request.target_name = target_name
        request.camera_name = camera_name
        while not rospy.is_shutdown():
            # get the target pose
            response = self.find_targets(request)
            if len(response.target.poses) == 0:
                if not self.yumi.check_command('Got empty reply. Try again?'):
                    print('Cancelled action. Exiting.')
                    exit()
            else:
                if self.yumi.check_command('Satisfied with the result?'):
                    break
        target_poses = []
        for pose in response.target.poses:
            target_poses.append(pose_msg_to_list(pose))
        return target_poses, response.confidence.data

    def pub_hand_poses(self, hand, pose):
        self.pm.hand_poses[self.pm.gripper_dict[hand]] = pose
        poses = PoseArray()
        for e in self.pm.hand_poses:
            poses.poses.append(list_to_pose_msg(e))
        poses.header.frame_id = self.yumi.robot_frame
        self.hand_pose_pub.publish(poses)

    def pub_aglet_pose(self, aglet, pose):
        self.pm.aglet_poses[aglet] = pose
        poses = PoseArray()
        for a in self.pm.aglet_poses.values():
            poses.poses.append(list_to_pose_msg(a))
        poses.header.frame_id = self.yumi.robot_frame
        self.aglet_pose_pub.publish(poses)

    def update_eyelet_poses(self, pose, id):
        if len(self.pm.eyelet_poses)>id:
            self.pm.eyelet_poses[id] = pose

    def update_cursor(self, name, id):
        if name=='left':
            self.pm.left_cursor = id
            self.cursor_pub.publish(Int32MultiArray(data=[id, self.pm.right_cursor]))
        elif name=='right':
            self.pm.right_cursor = id
            self.cursor_pub.publish(Int32MultiArray(data=[self.pm.left_cursor, id]))
        else:
            print('Unknown cursor name!')

    def pub_eyelet_poses(self):
        poses = PoseArray()
        for e in self.pm.eyelet_poses:
            poses.poses.append(list_to_pose_msg(e))
        poses.header.frame_id = self.yumi.robot_frame
        self.eyelet_pose_pub.publish(poses)

    def get_aglet_poses(self, aglet):
        side = self.pm.check_aglet_location(aglet)%2 # 0 for left, 1 for right
        if side==0:
            self.yumi.left_go_grasp()
        else:
            self.yumi.right_go_grasp()
        # get the target pose
        target_poses, _ = self.call_vision_srv(aglet, 'l515')
        # post processing
        target = target_poses[0]
        target = ls_add(target, (self.pm.l_l_offset if side==0 else self.pm.l_r_offset)+[0,0,0,0])
        [_, _, yaw] = euler_from_quaternion(target[3:]) # RPY
        # publish aglet pose
        self.pub_aglet_pose(aglet, target)
        return target, yaw

    def call_vision_eyelet(self, name):
        # get the position of the eyelet
        if name == 'eyelets_b':
            eyelet_poses, confidence = self.call_vision_srv(name, 'd435_l')
        elif name == 'eyelets_r':
            eyelet_poses, confidence = self.call_vision_srv(name, 'd435_r')
        else:
            print("Unknown eyelet name!")
        # pose processing
        eyelets = []
        for id, eyelet in enumerate(eyelet_poses):
            eyelet[3:] = list(euler_from_quaternion(eyelet[3:]))
            if name == 'eyelets_r':
                eyelet = ls_add(eyelet, self.pm.e_r_offset+[0,0,0])
            elif name == 'eyelets_b':
                eyelet = ls_add(eyelet, self.pm.e_l_offset+[0,0,0])
            if confidence[id]<0.6: # impossible, depth must be wrong
                eyelet[3:] = [0, pi/24*2, -pi/2] if name=='eyelets_b' else [0, pi/24*2, pi/2]
            eyelets.append(eyelet)
        if name == 'eyelets_b' and self.pm.left_cursor==0:
            eyelets[0][3:] = [0, pi/24*2, -pi/2]
        return np.array(eyelets), confidence

    def sanity_check_eyelets(self, eyelets, target_id):
        if any(np.isnan(eyelets[target_id][:3])):
            return False
        if len(eyelets) > 1: # do not test when only one is detected
            return is_sorted(eyelets[:,0]) and is_sorted(eyelets[:,2]) # x and z should be ascending
        else:
            return True
        
    def get_eyelet_poses(self, name, target_id=0, fine=False, init=True, compare_with=None):
        ''' 
            eyelet_id: index in all eyelets
            target_id: index on one side
            return: list of translations and euelr angles 
        '''
        # go to observe pose if first time
        if name=='eyelets_b' and init:
            self.yumi.left_go_observe()
        elif name=='eyelets_r' and init:
            self.yumi.right_go_observe(cartesian=True)
        eyelet_id = target_id*2 if name == 'eyelets_b' else target_id*2+1

        max_n_tryouts = 10
        rospy.sleep(0.5)
        # sanity check
        cursor = self.pm.left_cursor if name == 'eyelets_b' else self.pm.right_cursor
        target_id_visible = 0 if eyelet_id==cursor else target_id
        if self.debug: target_id_visible=target_id
        for _ in range(max_n_tryouts):
            eyelets, confidence = self.call_vision_eyelet(name)
            if not self.sanity_check_eyelets(eyelets, target_id_visible): # x and z should be ascending
                print('Result pose list does not seem to be correct. Retrying ...')
                rospy.sleep(0.1)
                continue
            elif compare_with is not None:
                if distance.euclidean(compare_with[:3], eyelets[target_id_visible][:3])<=0.01 or self.sim:
                    break
                else:
                    print('Camera depth faulty.')
                    continue
            else:
                break

        # check confidence and adjust observing position
        if fine and confidence[target_id_visible]<0.90: # first eyelet confidence low
            print('Start Active Observation for eyelet at {} with confidence {}!\
                  '.format(eyelets[target_id_visible], confidence[target_id_visible]))
            arm = 'left' if name=='eyelets_b' else 'right'
            base_to_eyelet = compose_matrix(translate=eyelets[target_id_visible][:3], angles=eyelets[target_id_visible][3:])

            image_offset = [-0.17, -0.03, -0.05, 0, -pi/2, 0] if arm=='left' else [-0.17, -0.03, -0.05, 0, -pi/2, 0]
            waypoints = [tf_mat2ls(base_to_eyelet@tf_ls2mat(image_offset))] # align eyelet to image centre
            self.yumi.tip_go_thro(arm, waypoints, "Active Observe")
            rospy.sleep(0.5) # wait for auto white balance
            eyelets = self.get_eyelet_poses(name,target_id=target_id,init=False)
        else:
            print('Target eyelet: {}, confidence {}'.format(eyelets[target_id_visible], confidence[target_id_visible]))
        self.update_eyelet_poses(eyelets[target_id_visible], eyelet_id)
        self.pub_eyelet_poses()
        return eyelets

    def update_aglet_ownership(self, aglet, site):
        self.pm.aglet_at[aglet] = site
        self.yumi.aglet_at[aglet] = site
        owner_msg = Int32MultiArray()
        owner_msg.data = [self.pm.sites_dict[self.pm.aglet_at['aglet_a']]-6, 
                        self.pm.sites_dict[self.pm.aglet_at['aglet_b']]-6] # left 0, right 1
        self.aglet_owner_pub.publish(owner_msg)

    def get_aglet_at(self, site):
        for a, s in self.pm.aglet_at.items():
            if s == site: return a
        else:
            return None

    def add_to_log(self, content):
        self.logs_pub.publish(String(content))
        rospy.sleep(0.5)

    def init_eyelet_poses(self):
        eyelets_b = self.get_eyelet_poses('eyelets_b') # left side
        eyelets_b = self.get_eyelet_poses('eyelets_b', target_id=len(eyelets_b)//2, fine=True) # left side
        self.yumi.left_go_observe()
        eyelets_r = self.get_eyelet_poses('eyelets_r') # right side
        eyelets_r = self.get_eyelet_poses('eyelets_r', target_id=len(eyelets_r)//2, fine=True) # right side
        self.yumi.right_go_observe()
        assert len(eyelets_b) == len(eyelets_r), 'Found {} eyelets on the left and {} on the right.\
            '.format(len(eyelets_b), len(eyelets_r))
        self.pm.load_eyelet_poses((eyelets_b, eyelets_r))

        self.pub_eyelet_poses() # publish eyelet poses
        self.yumi.both_go_grasp()
        self.get_aglet_poses('aglet_a')
        self.get_aglet_poses('aglet_b')

    def stop(self):
        self.yumi.stop()
        self.pm.save_params()


if __name__ == "__main__":
    rospy.init_node('sl_ctrl', anonymous=True)