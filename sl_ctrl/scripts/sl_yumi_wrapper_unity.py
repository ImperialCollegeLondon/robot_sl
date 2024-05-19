from math import pi
import threading
import numpy as np
from yumi_ctrl import YumiCtrl
from utils import tf_mat2ls, tf_ls2mat

import rospy
from sl_msgs.srv import *
from std_msgs.msg import Bool, String, Float32
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import MoveGroupActionFeedback
from moveit_msgs.msg import PositionConstraint, Constraints
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import compose_matrix, euler_matrix, euler_from_matrix


class YuMiWrapper(YumiCtrl):
    def __init__(self, auto_execution, workspace, vel_scale=1.0, gp_opening=0, table_offset=0, 
                 grasp_states=None, grasp_states2=None, observe_states=None):
        # super().__init__(auto_execution)
        super().__init__(auto_execution)

        self.POS_TOLERANCE = 1e-2
        self.TIME_TOLERANCE = 50
        self.GRIPPER_TOLERANCE = 2
        self.GRIPPER_TIME_TOLERANCE = 10
        # initialise the parameters
        self.auto_execution = auto_execution
        self.vel_scale = vel_scale
        self.gp_opening = gp_opening
        self.table_offset = table_offset
        self.workspace = workspace
        self.marker = True
        self.grasp_states = grasp_states
        self.grasp_states2 = grasp_states2
        self.observe_states = observe_states
        self.moveit_status = True # moveit ready by default
        self.side_thread_status = True # side thread ready by default
        self.aglet_to_gripper_l = compose_matrix(translate=[0, 0, self.gripper_offset], angles=[pi, 0, -pi/2])
        self.aglet_to_gripper_r = compose_matrix(translate=[0, 0, self.gripper_offset], angles=[pi, 0, pi/2])
        self.gripper_l_to_aglet = compose_matrix(translate=[0, 0, self.gripper_offset], angles=[pi, 0, -pi/2])
        self.gripper_r_to_aglet = compose_matrix(translate=[0, 0, self.gripper_offset], angles=[pi, 0, pi/2])
        self.shoe_rotation = None
        self.shoe_translation = None

        # start initialisation
        self.aglet_at = {"aglet_a":"site_l1", "aglet_b":"site_r1"}
        self.sl_constraint_a = Constraints()
        self.sl_constraint_b = Constraints()
        self.add_collision_objs(workspace)
        self.closeLeftGripper(True)
        print('left gripper position set')
        self.closeRightGripper(True)
        print('right gripper position set')
        self.times_grasping_l = 0
        self.times_grasping_r = 0
        if self.observe_states:
            super().go_to_angles_both(observe_states, 'Observe', velocity_scaling=vel_scale, wait=True)
        print('arms position set')

        # register subscriber and publishers
        self.arm_l_speed_pub = rospy.Publisher("yumi/left_arm/speed_ratio", Float32, queue_size=1)
        self.arm_r_speed_pub = rospy.Publisher("/yumi/right_arm/speed_ratio", Float32, queue_size=1)
        rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.moveit_feedback_cb, queue_size=1)
        self.sl_constriant_marker_pub = rospy.Publisher('sl_constriant', Marker, queue_size=10)
        self.logs_pub = rospy.Publisher('sl_logs', String, queue_size=1)

        # create and start the side thread
        self.side_queue = []
        self.side_thread = threading.Thread(target=self.side_thread_func, daemon=True)
        self.side_thread.start()

    def side_thread_func(self):
        ''' The side thread manages actions that do not require immediate implementation '''
        while True:
            try:
                if len(self.side_queue)>0:
                    action = self.side_queue.pop(0)
                    print(f'executing side thread{action[0]}')
                    action[0](*action[1])
                    print(f'side thread {action[0]} finished')
                else:
                    self.side_thread_status = True
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(e)

    ### Contact mode ###
    def go_touch_rel(self, arm, offset, duration):
        if 'l' in arm:
            self.left_go_touch_rel(offset, duration)
        elif 'r' in arm:
            self.right_go_touch_rel(offset, duration)
        else:
            print('Unknown Arm!')
            exit()

    ### Tip ###
    def tip_go_to(self, arm, target_tip, statement, velocity_scaling=None, main=True):
        if 'l' in arm:
            self.left_tip_go_to(target_tip, statement, velocity_scaling, main)
        elif 'r' in arm:
            self.right_tip_go_to(target_tip, statement, velocity_scaling, main)
        else:
            print('Unknown Arm!')
            exit()
            
    def left_tip_go_to(self, target_tip, statement, velocity_scaling=None, main=True):
        ''' Set target in finger tip frame rather than ee frame '''
        target_mat = tf_ls2mat(target_tip)
        target = tf_mat2ls(target_mat@self.aglet_to_gripper_l)
        self.left_go_to(target, statement, velocity_scaling=velocity_scaling, main=main)
        # _, _, target_euler, target, _ = decompose_matrix(target_mat@self.aglet_to_gripper_l)
        # self.left_go_to(np.concatenate((target, target_euler)), statement, velocity_scaling=velocity_scaling, main=main)

    def right_tip_go_to(self, target_tip, statement, velocity_scaling=None, main=True):
        ''' Set target in finger tip frame rather than ee frame '''
        target_mat = tf_ls2mat(target_tip)
        target = tf_mat2ls(target_mat@self.aglet_to_gripper_r)
        self.right_go_to(target, statement, velocity_scaling=velocity_scaling, main=main)
        # _, _, target_euler, target, _ = decompose_matrix(target_mat@self.aglet_to_gripper_r)
        # self.right_go_to(np.concatenate((target, target_euler)), statement, velocity_scaling=velocity_scaling, main=main)

    def tip_go_thro(self, arm, points_tip, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=None, main=True):
        if 'l' in arm:
            self.left_tip_go_thro(points_tip, statement, eef_step, jump_threshold, velocity_scaling, main)
        elif 'r' in arm:
            self.right_tip_go_thro(points_tip, statement, eef_step, jump_threshold, velocity_scaling, main)
        else:
            print('Unknown Arm!')
            exit()

    def left_tip_go_thro(self, points_tip, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=None, main=True):
        ''' Set target in finger tip frame rather than ee frame '''
        points = []
        for p_tip in points_tip:
            target_mat = tf_ls2mat(p_tip)
            points.append(tf_mat2ls(target_mat@self.aglet_to_gripper_l))
            # _, _, target_euler, target, _ = decompose_matrix(target_mat@self.aglet_to_gripper_l)
            # points.append(np.concatenate((target, target_euler)))
        self.left_go_thro(points, statement, eef_step=eef_step, jump_threshold=jump_threshold, velocity_scaling=velocity_scaling, main=main)

    def right_tip_go_thro(self, points_tip, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=None, main=True):
        ''' Set target in finger tip frame rather than ee frame '''
        points = []
        for p_tip in points_tip:
            target_mat = tf_ls2mat(p_tip)
            points.append(tf_mat2ls(target_mat@self.aglet_to_gripper_r))
            # _, _, target_euler, target, _ = decompose_matrix(target_mat@self.aglet_to_gripper_r)
            # points.append(np.concatenate((target, target_euler)))
        self.right_go_thro(points, statement, eef_step=eef_step, jump_threshold=jump_threshold, velocity_scaling=velocity_scaling, main=main)


    ### EE ###
    def left_go_to(self, target, statement, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.left_arm_group)
        ''' Override left_go_to for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.left_go_to, [target, statement, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_l()
        super().left_go_to(target[:3], target[3:], statement, velocity_scaling=velocity_scaling, marker=self.marker, wait=True)
        self.remove_constraints(self.left_arm_group)

    def right_go_to(self, target, statement, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.right_arm_group)
        ''' Override right_go_to for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.right_go_to, [target, statement, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_r()
        super().right_go_to(target[:3], target[3:], statement, velocity_scaling=velocity_scaling, marker=self.marker, wait=True)
        self.remove_constraints(self.right_arm_group)

    def left_go_thro(self, points, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.left_arm_group)
        ''' Override left_go_thro for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.left_go_thro, [points, statement, eef_step, jump_threshold, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_l()
        super().left_go_thro(points, statement, eef_step=eef_step, jump_threshold=jump_threshold, 
            velocity_scaling=velocity_scaling, marker=self.marker, wait=True)
        self.remove_constraints(self.left_arm_group)

    def right_go_thro(self, points, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.right_arm_group)
        ''' Override right_go_thro for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.right_go_thro, [points, statement, eef_step, jump_threshold, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_r()
        super().right_go_thro(points, statement, eef_step=eef_step, jump_threshold=jump_threshold, 
            velocity_scaling=velocity_scaling, marker=self.marker, wait=True)
        self.remove_constraints(self.right_arm_group)
    
    def go_to_angles_left(self, joint_position_array, statement, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.left_arm_group)
        ''' Override go_to_angles_left for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.go_to_angles_left, [joint_position_array, statement, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_l()
        super().go_to_angles_left(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=True)
        self.remove_constraints(self.left_arm_group)

    def go_to_angles_right(self, joint_position_array, statement, velocity_scaling=None, main=True, override_const=False):
        if not override_const:
            self.set_constraints(self.right_arm_group)
        ''' Override go_to_angles_right for main and side thread separation '''
        self.wait_for_moveit()
        if not main:
            self.side_thread_status = False
            self.side_queue.append([self.go_to_angles_right, [joint_position_array, statement, velocity_scaling]])
            return
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        self.wait_for_completion_r()
        super().go_to_angles_right(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=True)
        self.remove_constraints(self.right_arm_group)

    def go_to_angles_both(self, joint_position_array, statement, velocity_scaling=None, override_const=False):
        if not override_const:
            self.set_constraints(self.left_arm_group)
            self.set_constraints(self.right_arm_group)
        ''' Override go_to_angles_both for main and side thread separation '''
        self.wait_for_moveit()
        self.wait_for_completion_l()
        self.wait_for_completion_gripper_l()
        self.wait_for_completion_r()
        self.wait_for_completion_gripper_r()
        if velocity_scaling is None: velocity_scaling=self.vel_scale
        super().go_to_angles_both(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=True)
        self.remove_constraints(self.left_arm_group)
        self.remove_constraints(self.right_arm_group)


    ### Pre-defined ###
    def both_go_grasp(self):
        self.go_to_angles_both(self.grasp_states, 'Pre-grasp')
    def left_go_grasp(self, main=True):
        self.go_to_angles_left(self.grasp_states[:7], 'Pre-grasp', main=main)
    def left_go_grasp2(self, main=True):
        self.go_to_angles_left(self.grasp_states2[:7], 'Pre-grasp', main=main)
    def left_go_observe(self, main=True):
        self.go_to_angles_left(self.observe_states[:7], 'Observe', main=main)
    def left_go_transfer(self, main=True):
        self.left_tip_go_thro([[0.45, 0, 0.15, 0, 0, -pi/2]], 'Transfer')

    def right_go_grasp(self, main=True):
        self.go_to_angles_right(self.grasp_states[7:], 'Pre-grasp', main=main)
    def right_go_grasp2(self, main=True):
        self.go_to_angles_right(self.grasp_states2[7:], 'Pre-grasp', main=main)
    def right_go_observe(self, main=True, cartesian=False):
        if cartesian:
            # self.right_go_thro([[0.5, -0.28, 0.2, -pi/24*7, pi, 0]], 'Observe')
            self.right_go_thro([[0.45, -0.33, 0.25, -pi/24*7, pi, 0]], 'Observe')
        else:
            self.go_to_angles_right(self.observe_states[7:], 'Observe', main=main)
    def right_go_transfer(self, main=True):
        self.right_tip_go_thro([[0.45, 0, 0.15, 0, 0, pi/2]], 'Transfer')


    ### Grippers ###
    def open_gripper(self, arm, full=0, main=True):
        if 'l' in arm:
            self.open_left_gripper(full, main)
        elif 'r' in arm:
            self.open_right_gripper(full, main)
        else:
            print('Unknown Arm!')
            exit()

    def open_left_gripper(self, full=0, main=True):
        ''' open gripper to pre-specified width unless override by setting full to True '''
        if not main:
            self.side_queue.append([self.open_left_gripper, [full]])
            return
        if full==1: # fully open
            self.openLeftGripper(True)
        elif full==2: # half opening open
            self.setLeftGripperPos(self.gp_opening-2.5, True) # TODO 2.5 is half of the offset
        else: # full opening open
            self.setLeftGripperPos(self.gp_opening, True)

    def open_right_gripper(self, full=0, main=True):
        ''' open gripper to pre-specified width unless override by setting full to True '''
        if not main:
            self.side_queue.append([self.open_right_gripper, [full]])
            return
        if full==1: # fully open
            self.openRightGripper(True)
        elif full==2: # half opening open
            self.setRightGripperPos(self.gp_opening-2.5, True) # TODO 2.5 is half of the offset
        else:
            self.setRightGripperPos(self.gp_opening, True)

    def close_gripper(self, arm, full_force=True, main=True):
        if 'l' in arm:
            self.close_left_gripper(full_force, main)
        elif 'r' in arm:
            self.close_right_gripper(full_force, main)
        else:
            print('Unknown Arm!')
            exit()

    def close_left_gripper(self, full_force=True, main=True):
        ''' close gripper with full force unless specified with full_force parameter '''
        self.times_grasping_l+=1
        self.logs_pub.publish(String("|# Grasping L| "+str(self.times_grasping_l)))
        if not main:
            self.side_queue.append([self.close_left_gripper, [full_force]])
            return
        if full_force:
            self.sendLeftGripperCmd(19, True)
            rospy.sleep(1)
        else:
            self.closeLeftGripper(True)
            rospy.sleep(1)

    def close_right_gripper(self, full_force=True, main=True):
        ''' close gripper with full force unless specified with full_force parameter '''
        self.times_grasping_r+=1
        self.logs_pub.publish(String("|# Grasping R| "+str(self.times_grasping_r)))
        if not main:
            self.side_queue.append([self.close_right_gripper, [full_force]])
            return
        if full_force:
            self.sendRightGripperCmd(19, True)
            rospy.sleep(1)
        else:
            self.closeRightGripper(True)
            rospy.sleep(1)

    def wait_for_side_thread(self):
        ''' wait for the side thread to complete '''
        while not self.side_thread_status:
            rospy.sleep(0.1)

    def wait_for_moveit(self):
        ''' wait for moveit to be ready '''
        while not rospy.is_shutdown():
            if self.moveit_status:
                break
            else:
                rospy.sleep(0.01)
    
    def wait_for_contact(self, arm):
        if 'l' in arm:
            return rospy.wait_for_message('/yumi/left_arm/in_contact', Bool).data
        elif 'r' in arm:
            return rospy.wait_for_message('/yumi/right_arm/in_contact', Bool).data
        else:
            print('Unknown Arm!')
            exit()

    def moveit_feedback_cb(self, msg):
        ''' register move group status '''
        self.moveit_status = msg.feedback.state == 'IDLE'

    def change_speed(self, speed_ratio):
        ''' change speed ratio (in percentage of the max speed) '''
        self.arm_l_speed_pub.publish(Float32(speed_ratio))
        self.arm_r_speed_pub.publish(Float32(speed_ratio))

    ### Planning Scene ###
    def update_sl_constriants(self, branch, sl_length, anchor):
        pcm = PositionConstraint()
        # pcm.link_name = self.left_arm_group.get_end_effector_link() if 'l' in group else self.right_arm_group.get_end_effector_link()
        pcm.header.frame_id = self.robot_frame
        pcm.target_point_offset = Point(0.0, 0.0, self.gripper_offset) # constraint on the tip
        pcm.weight = 1.0

        boundary = SolidPrimitive()
        boundary.type = boundary.SPHERE
        boundary.dimensions = [sl_length*1.01] # slack off 1 percent
        # boundary.dimensions[boundary.SPHERE_RADIUS] = sl_length
        pcm.constraint_region.primitives.append(boundary)

        boundary_pose = Pose()
        boundary_pose.position = Point(*anchor)
        boundary_pose.orientation = Quaternion(0,0,0,1)
        pcm.constraint_region.primitive_poses.append(boundary_pose)

        if 'aglet_a' == branch:
            self.sl_constraint_a = pcm
            # self.plot_sl_constriant_marker(pcm, 0)
        elif 'aglet_b' == branch:
            self.sl_constraint_b = pcm
            # self.plot_sl_constriant_marker(pcm, 1)

    def set_constraints(self, group):
        gripper = 'left_gripper' if group==self.left_arm_group else 'right_gripper'
        if self.aglet_at['aglet_a'] == gripper:
            constraint = self.sl_constraint_a
        elif self.aglet_at['aglet_b'] == gripper:
            constraint = self.sl_constraint_b
        else:
            return
        constraint.link_name = group.get_end_effector_link()
        moveit_constraints = Constraints()
        moveit_constraints.position_constraints.append(constraint)
        # group.set_path_constraints(moveit_constraints)

    def plot_sl_constriant_marker(self, constraint, id=0):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position = constraint.constraint_region.primitive_poses[0].position
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.scale.x = constraint.constraint_region.primitives[0].dimensions[0]*2
        marker.scale.y = constraint.constraint_region.primitives[0].dimensions[0]*2
        marker.scale.z = constraint.constraint_region.primitives[0].dimensions[0]*2
        marker.color.a = 0.3
        marker.color.r = 1.0 if id == 0 else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if id == 0 else 1.0
        marker.lifetime = rospy.Duration(0) # 0 means forever
        self.sl_constriant_marker_pub.publish(marker)

    def remove_constraints(self, group):
        group.clear_path_constraints()
        # self.remove_all_markers()

    def remove_all_markers(self):
        marker = Marker()
        marker.id = 0
        # marker.ns = self.marker_ns
        marker.action = Marker.DELETEALL
        self.sl_constriant_marker_pub.publish(marker)

    def add_collision_objs(self, ws):
        ''' Add table and walls to the MoveIt planing scene '''
        self.table_box = [[ws[1]/2, (ws[3]+ws[2])/2, self.table_offset], [ws[1], ws[3]-ws[2], .01]]
        self.add_collision_box('table', self.table_box[0], self.table_box[1])
        self.add_collision_box('wall1', [ws[1]/2, ws[2], (ws[5]+ws[4])/2], [ws[1], .01, ws[5]-ws[4]])
        self.add_collision_box('wall2', [ws[1]/2, ws[3], (ws[5]+ws[4])/2], [ws[1], .01, ws[5]-ws[4]])
        self.ceiling_box = [[ws[1]/2, (ws[3]+ws[2])/2, ws[5]], [ws[1], ws[3]-ws[2], .01]]
        self.add_collision_box('ceiling', self.ceiling_box[0], self.ceiling_box[1])
        # self.yumi.add_collision_box('wall3', [ws[1], (ws[3]+ws[2])/2, (ws[5]+ws[4])/2], [.01, ws[3]-ws[2], ws[5]-ws[4]])

    def remove_table(self):
        ''' Remove the table and shoe model from the MoveIt planing scene '''
        self.remove_collision_objects('table')

    def add_table(self):
        ''' Add the table from the MoveIt planing scene '''
        self.add_collision_box('table', self.table_box[0], self.table_box[1])

    ### Utilities ###
    def ee_rot_to_tip_l(self, euler):
        g2b = euler_matrix(*euler)
        a2g = euler_matrix(*[pi, 0, -pi/2])
        a2b = np.matmul(g2b, a2g)
        return list(euler_from_matrix(a2b))

    def ee_rot_to_tip_r(self, euler):
        g2b = euler_matrix(*euler)
        a2g = euler_matrix(*[pi, 0, pi/2])
        a2b = np.matmul(g2b, a2g)
        return list(euler_from_matrix(a2b))
    
if __name__ == '__main__':
    rospy.init_node('yumi_wrapper', anonymous=True)
    yumi = YuMiWrapper(auto_execution=True, workspace=[-0.5, 1.0, -0.75, 0.75, 0.0, 0.8], grasp_states=[-0.6677426099777222, -1.143572211265564, 1.6579793691635132, 0.6495829820632935,
  2.425766944885254, 0.5661510825157166, 2.1443932056427, 0.6723159551620483, -1.0775535106658936,
  -1.690792202949524, 0.6427130103111267, 3.8197340965270996, 0.521094024181366, -2.0585293769836426], observe_states=[-1.2257874011993408, -0.4985324442386627, 1.980921745300293, -0.058894701302051544,
  -0.47677695751190186, 0.9440299868583679, -2.054021120071411, 1.3764188289642334,
  -0.5266491770744324, -2.042018175125122, -0.25265806913375854, 0.39488810300827026,
  1.015584111213684, 2.192312002182007])
    yumi.left_go_grasp()