import sys
import time
import yaml
import signal
import numpy as np
from math import pi, sqrt
from random import random
from copy import deepcopy

import rospy
import moveit_commander
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float64, Bool, String, Float64MultiArray, MultiArrayDimension
from industrial_msgs.srv import StopMotionRequest, StopMotion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point
from moveit_msgs.msg import PositionConstraint, DisplayTrajectory, Constraints, OrientationConstraint
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class YumiCtrl:

    auto_execution = False
    # gripper_offset = 0.136
    gripper_offset = 0.137+0.02175
    robot_frame = "yumi_base_link"
    ee_frame_l = 'yumi_link_7_l'
    ee_frame_r = 'yumi_link_7_r'
    marker_topic = '/visualization_marker'
    stop_topic = 'yumi_ctrl/stop'
    POS_TOLERANCE = 1e-4 # industrial_core thresh: 0.01
    GRIPPER_TOLERANCE = 0.5 # mm
    TIME_TOLERANCE = 50
    GRIPPER_TIME_TOLERANCE = 5
    SINGLE_MOTION_TIME_CAP = 3

    vel_limits = [pi, pi, pi, pi, 22*pi, 22*pi, 22*pi] # urdf index, same for both arms

    left_gripper_cmd_pub = rospy.Publisher('/yumi/gripper_l_effort_cmd', Float64, queue_size=1)
    right_gripper_cmd_pub = rospy.Publisher('/yumi/gripper_r_effort_cmd', Float64, queue_size=1)
    left_gripper_pos_pub = rospy.Publisher('/yumi/gripper_l_position_cmd', Float64, queue_size=1)
    right_gripper_pos_pub = rospy.Publisher('/yumi/gripper_r_position_cmd', Float64, queue_size=1)

    left_arm_traj_pub = rospy.Publisher('/yumi/left_arm/joint_path_command', JointTrajectory, queue_size=1)
    right_arm_traj_pub = rospy.Publisher('/yumi/right_arm/joint_path_command', JointTrajectory, queue_size=1)

    left_arm_stop_pub = rospy.Publisher('/yumi/left_arm/joint_trajectory_action/cancel', GoalID, queue_size=1)
    right_arm_stop_pub = rospy.Publisher('/yumi/right_arm/joint_trajectory_action/cancel', GoalID, queue_size=1)

    left_cart_target_pub = rospy.Publisher('yumi/left_arm/cartesian_target', Float64MultiArray, queue_size=10)
    right_cart_target_pub = rospy.Publisher('yumi/left_arm/cartesian_target', Float64MultiArray, queue_size=10)

    def __init__(self, auto_execution=False, logger=None):
        self.auto_execution = auto_execution
        # initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)

        self.yumi = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        self.both_arm_group = moveit_commander.MoveGroupCommander("both_arms")
        # check yumi_moveit_config/config/ompl_planning.yaml for other planners
        self.set_planner_params(self.left_arm_group, 'RRTConnectkConfigDefault', 2, True)
        self.set_planner_params(self.right_arm_group, 'RRTConnectkConfigDefault', 2, True)
        self.set_planner_params(self.both_arm_group, 'RRTConnectkConfigDefault', 2, True)
        
        # initialise publishers
        self.log = logger is not None
        if logger is not None:
            self.logs_pub = rospy.Publisher(logger, String, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                         DisplayTrajectory,
        #                                         queue_size=20)

        # initialise subscribers
        rospy.Subscriber('/joint_states', JointState, self.yumi_states_sb, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self.stop_cb, queue_size=1)

        # initialise service clients
        self.left_arm_stop_srv = rospy.ServiceProxy('/yumi/left_arm/stop_motion', StopMotion)
        self.right_arm_stop_srv = rospy.ServiceProxy('/yumi/right_arm/stop_motion', StopMotion)
        
        self.left_arm_state = rospy.wait_for_message("/yumi/left_arm/joint_states", JointState).position
        self.left_arm_target = self.to_urdf_index(self.left_arm_state)
        self.right_arm_state = rospy.wait_for_message("/yumi/right_arm/joint_states", JointState).position
        self.right_arm_target = self.to_urdf_index(self.right_arm_state)
        self.left_gripper_state = rospy.wait_for_message("/yumi/gripper_states", JointState).position[0]*1000
        self.left_gripper_target = self.left_gripper_state
        self.right_gripper_state = rospy.wait_for_message("/yumi/gripper_states", JointState).position[1]*1000
        self.right_gripper_target = self.right_gripper_state

        # set constriants
        # rospy.sleep(2)
        # self.setConstriants(self.left_arm_group)
        # self.setConstriants(self.right_arm_group)

        signal.signal(signal.SIGINT, self.signal_term_handler)

    def set_planner_params(self, group, planner, num_attempts, enable_replanning):
        group.set_planner_id(planner)
        group.set_num_planning_attempts(num_attempts)
        group.allow_replanning(enable_replanning)
        # group.set_end_effector_link("")
        # group.set_goal_position_tolerance(0.01)
        # group.set_goal_orientation_tolerance(0.01)

    def signal_term_handler(self, signal, frame):
        '''Handles KeyboardInterrupts to ensure smooth exit'''
        rospy.logwarn('User Keyboard interrupt')
        sys.exit(0)

    def both_go_to(self, pose_l, pose_r, statement, velocity_scaling=1.0, wait=True):
        """ send left and right arm to target posese simutaneously """
        self.both_arm_group.set_pose_target(pose_l, end_effector_link=self.ee_frame_l)
        self.both_arm_group.set_pose_target(pose_r, end_effector_link=self.ee_frame_r)
        (success, plan, _, _) = self.both_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions[:7]
            self.right_arm_target = plan.joint_trajectory.points[-1].positions[7:]
            self.both_arm_group.execute(plan, wait=wait)
            if wait:
                self.wait_for_completion_l()
                self.wait_for_completion_r()
        elif command == None:
            self.both_go_to(pose_l, pose_r, statement, velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def left_go_to(self, target, statement, velocity_scaling=1.0, marker=True, wait=True):
        """ send left arm to target pose """
        self.left_arm_group.stop()
        self.left_arm_group.set_start_state_to_current_state()
        pose_goal = Pose()
        pose_goal.position = Point(*target[:3])
        if len(target) == 6:
            pose_goal.orientation = Quaternion(*quaternion_from_euler(*target[3:]))
        elif len(target) == 7:
            pose_goal.orientation = Quaternion(*target[3:])
        else:
            rospy.logwarn("Incorrect orientation length!")
        (success, plan, _, _) = self.left_arm_group.plan(pose_goal)
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        if marker:
            self.plot_marker(position=target[:3], orientation=target[3:])
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions
            # self.left_arm_group.execute(plan, wait=wait)
            self.left_arm_execute(plan)
            if wait:
                self.wait_for_completion_l()
        elif command == None:
            self.left_go_to(target, statement, velocity_scaling, marker=marker, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def right_go_to(self, target, statement, velocity_scaling=1.0, marker=True, wait=True):
        """ send right arm to target pose """
        self.right_arm_group.stop()
        self.right_arm_group.set_start_state_to_current_state()
        pose_goal = Pose()
        pose_goal.position = Point(*target[:3])
        if len(target) == 6:
            pose_goal.orientation = Quaternion(*quaternion_from_euler(*target[3:]))
        elif len(target) == 7:
            pose_goal.orientation = Quaternion(*target[3:])
        else:
            rospy.logwarn("Incorrect orientation length!")
        (success, plan, _, _) = self.right_arm_group.plan(pose_goal)
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        if marker:
            self.plot_marker(position=target[:3], orientation=target[3:])
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.right_arm_target = plan.joint_trajectory.points[-1].positions
            # self.right_arm_group.execute(plan, wait=wait)
            self.right_arm_execute(plan)
            if wait:
                self.wait_for_completion_r()
        elif command == None:
            self.right_go_to(target, statement, velocity_scaling, marker=marker, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def left_go_thro(self, points, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=1.0, acceleration_scaling=1.0, marker=True, wait=True):
        """ send left arm to go through a couple of way points """
        self.left_arm_group.stop()
        self.left_arm_group.set_start_state_to_current_state()
        way_points = []
        for point in points:
            way_point = Pose()
            if len(point) == 7:
                way_point.orientation = Quaternion(*point[3:])
            elif len(point) == 6:
                way_point.orientation = Quaternion(*quaternion_from_euler(*point[3:]))
            else:
                rospy.logerr("Invalid way point shape! Got point {}".format(point))
                return
            way_point.position = Point(*point[:3])
            way_points.append(deepcopy(way_point))
        plan, fraction = self.left_arm_group.compute_cartesian_path(way_points, eef_step, jump_threshold)
        # plan = self.left_arm_group.retime_trajectory(self.yumi.get_current_state(), plan, velocity_scaling, acceleration_scaling)
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        # plan.joint_trajectory.points.pop(0) # remove the first waypoint whose velocity is 0
        if marker:
            self.plot_marker(position=point[:3], orientation=point[3:])
        command = self.check_command("[{}] Planning {} completed.".format(statement, fraction))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions
            # self.left_arm_group.execute(plan, wait=wait)
            self.left_arm_execute(plan)
            if wait:
                self.wait_for_completion_l()
        elif command == None:
            self.left_go_thro(points, statement, eef_step, jump_threshold, velocity_scaling, acceleration_scaling, marker, wait)
        else:
            rospy.logwarn("Action skipped")

    def right_go_thro(self, points, statement, eef_step=0.01, jump_threshold=2.0, velocity_scaling=1.0, acceleration_scaling=1.0, marker=True, wait=True):
        """ send right arm to go through a couple of way points """
        self.right_arm_group.stop()
        self.right_arm_group.set_start_state_to_current_state()
        way_points = []
        for point in points:
            way_point = Pose()
            if len(point) == 7:
                way_point.orientation = Quaternion(*point[3:])
            elif len(point) == 6:
                way_point.orientation = Quaternion(*quaternion_from_euler(*point[3:]))
            else:
                rospy.logerr("Invalid way point shape! Got point {}".format(point))
                return
            way_point.position = Point(*point[:3])
            way_points.append(deepcopy(way_point))
        plan, fraction = self.right_arm_group.compute_cartesian_path(way_points, eef_step, jump_threshold)
        # plan = self.right_arm_group.retime_trajectory(self.yumi.get_current_state(), plan, velocity_scaling, acceleration_scaling)
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        # print(plan)
        if marker:
            self.plot_marker(position=point[:3], orientation=point[3:])
        command = self.check_command("[{}] Planning {} completed.".format(statement, fraction))
        if command == True:
            self.right_arm_target = plan.joint_trajectory.points[-1].positions
            # self.right_arm_group.execute(plan, wait=wait)
            self.right_arm_execute(plan)
            if wait:
                self.wait_for_completion_r()
        elif command == None:
            self.right_go_thro(points, statement, eef_step, jump_threshold, velocity_scaling, acceleration_scaling, marker=marker, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def left_go_to_named(self, name, velocity_scaling=1.0, wait=True):
        """ send left arms to named positions """
        self.left_arm_group.set_named_target(name)
        (success, plan, _, _) = self.left_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        command = self.check_command("Planned for going to {}.".format(name))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions
            # self.left_arm_group.execute(plan, wait=wait)
            self.left_arm_execute(plan)
            if wait:
                self.wait_for_completion_l()
        elif command == None:
            self.left_go_to_named(name, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def right_go_to_named(self, name, velocity_scaling=1.0, wait=True):
        """ send right arms to named positions """
        self.right_arm_group.set_named_target(name)
        (success, plan, _, _) = self.right_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        command = self.check_command("Planned for going to {}.".format(name))
        if command == True:
            self.right_arm_target = plan.joint_trajectory.points[-1].positions
            # self.right_arm_group.execute(plan, wait=wait)
            self.right_arm_execute(plan)
            if wait:
                self.wait_for_completion_r()
        elif command == None:
            self.right_go_to_named(name, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def go_to_named(self, name, velocity_scaling=1.0, wait=True):
        """ send both arms to named positions """
        self.both_arm_group.set_named_target(name)
        (success, plan, _, _) = self.both_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        command = self.check_command("Planned for going to {}.".format(name))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions[:7]
            self.right_arm_target = plan.joint_trajectory.points[-1].positions[7:]
            self.both_arm_group.execute(plan, wait=wait)
            if wait:
                self.wait_for_completion_l()
                self.wait_for_completion_r()
        elif command == None:
            self.go_to_named(name, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def go_to_angles_left(self, joint_position_array, statement, velocity_scaling=1.0, wait=True):
        """ send the left arm to the disired angles """
        self.left_arm_group.stop()
        self.left_arm_group.set_start_state_to_current_state()
        self.left_arm_group.set_joint_value_target(joint_position_array)
        (success, plan, _, _) = self.left_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        # self.save_plan(plan, 'plan.yaml')
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions
            # self.left_arm_group.execute(plan, wait=wait)
            self.left_arm_execute(plan)
            if wait:
                self.wait_for_completion_l()
        elif command == None:
            self.go_to_angles_left(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def go_to_angles_right(self, joint_position_array, statement, velocity_scaling=1.0, wait=True):
        """ send the right arm to the disired angles """
        self.right_arm_group.stop()
        self.right_arm_group.set_start_state_to_current_state()
        self.right_arm_group.set_joint_value_target(joint_position_array)
        (success, plan, _, _) = self.right_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        # self.save_plan(plan, 'plan.yaml')
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.right_arm_target = plan.joint_trajectory.points[-1].positions
            # self.right_arm_group.execute(plan, wait=wait)
            self.right_arm_execute(plan)
            if wait:
                self.wait_for_completion_r()
        elif command == None:
            self.go_to_angles_right(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def go_to_angles_both(self, joint_position_array, statement, velocity_scaling=1.0, wait=True):
        """ send the right arm to the disired angles """
        self.both_arm_group.stop()
        self.both_arm_group.set_start_state_to_current_state()
        self.both_arm_group.set_joint_value_target(joint_position_array)
        (success, plan, _, _) = self.both_arm_group.plan()
        plan = self.plan_post_processing(plan, velocity_scaling=velocity_scaling)
        command = self.check_command("[{}] Planning completed.".format(statement))
        if command == True:
            self.left_arm_target = plan.joint_trajectory.points[-1].positions[:7]
            self.right_arm_target = plan.joint_trajectory.points[-1].positions[7:]
            self.both_arm_group.execute(plan, wait=wait)
            if wait:
                self.wait_for_completion_l()
                self.wait_for_completion_r()
        elif command == None:
            self.go_to_angles_both(joint_position_array, statement, velocity_scaling=velocity_scaling, wait=wait)
        else:
            rospy.logwarn("Action skipped")

    def left_go_touch_rel(self, pose, duration):
        '''
        pose: list with x,y,z (cannot change direction)
        duration: time to finish this movement in seconds
        This function sends the left gripper to a positional offset 
        provided by pose argment. If there is a collision, the 
        'in_contact' topic will change to True.
        '''
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [5]+pose+[0]*4+[duration]
        self.left_cart_target_pub.publish(target)
        rospy.sleep(duration+0.5)

    def right_go_touch_rel(self, pose, duration):
        '''
        pose: list with x,y,z (cannot change direction)
        duration: time to finish this movement in seconds
        This function sends the right gripper to a positional offset 
        provided by pose argment. If there is a collision, the 
        'in_contact' topic will change to True.
        '''
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [5]+pose+[0]*4+[duration]
        self.right_cart_target_pub.publish(target)
        rospy.sleep(duration+0.5)

    def left_lead_through_start(self):
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [10]+[0]*7+[0]
        self.left_cart_target_pub.publish(target)

    def left_lead_through_stop(self):
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [11]+[0]*7+[0]
        self.left_cart_target_pub.publish(target)

    def right_lead_through_start(self):
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [10]+[0]*7+[0]
        self.right_cart_target_pub.publish(target)

    def left_lead_through_stop(self):
        target = Float64MultiArray()
        target.layout.dim.append(MultiArrayDimension)
        target.layout.dim[0].label = 'target'
        target.layout.dim[0].size = 9
        target.layout.dim[0].stride = 9
        target.data = [11]+[0]*7+[0]
        self.right_cart_target_pub.publish(target)

    #############
    ### Utils ###
    #############

    def left_arm_execute(self, plan):
        # action_goal = FollowJointTrajectoryActionGoal()
        # goal = FollowJointTrajectoryGoal()
        # traj = plan.joint_trajectory
        # goal.trajectory = traj
        # action_goal.goal = goal
        self.left_arm_traj_pub.publish(plan.joint_trajectory)
        # return action_goal

    def right_arm_execute(self, plan):
        self.right_arm_traj_pub.publish(plan.joint_trajectory)

    def plan_post_processing(self, plan, velocity_scaling=1.0, acceleration_scaling=1.0):
        ''' Change the speed of the plan '''
        new_plan = deepcopy(plan)
        # new_plan.joint_trajectory.points = []
        # # only works with single arms
        # if len(plan.joint_trajectory.points[0].positions) == 7:
        #     # cap time to SINGLE_MOTION_TIME_CAP if requested
        #     total_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        #     if self.SINGLE_MOTION_TIME_CAP and total_time/velocity_scaling>self.SINGLE_MOTION_TIME_CAP:
        #         # print('########################################################')
        #         # print(total_time/velocity_scaling)
        #         velocity_scaling = total_time/self.SINGLE_MOTION_TIME_CAP
        #     # verify the scale
        #     vel_mat = [vel_list for vel_list in plan.joint_trajectory.points[-1].velocities]
        #     if len(vel_mat) == len(self.vel_limits):
        #         max_vel_ratio = np.max(np.array(vel_mat)*velocity_scaling/np.array(self.vel_limits))
        #     else:
        #         max_vel_ratio = 1
        #     if max_vel_ratio>=1: 
        #         rospy.logwarn('Velocity scale too high! Reduced to max limit.')
        #         velocity_scaling/=max_vel_ratio
        # for point in plan.joint_trajectory.points:
        #     drt = point.time_from_start.to_sec()/velocity_scaling # abb driver uses only the duration
        #     point.time_from_start = rospy.Duration.from_sec(drt)
        #     new_plan.joint_trajectory.points.append(point)
        return new_plan

    def check_vel_validity(self, vel_list):
        return all(a < b for (a, b) in zip(vel_list, self.vel_limits))

    def stop(self):
        """ Stop the movement """
        self.left_arm_group.stop()
        self.right_arm_group.stop()

    def stop_cb(self, msg):
        # self.left_arm_stop_pub.publish(GoalID())
        # self.right_arm_stop_pub.publish(GoalID())
        req = StopMotionRequest()
        self.left_arm_stop_srv(req)
        self.right_arm_stop_srv(req)

    def check_command(self, statement, override=False):
        """ wait for user instruction """
        if self.log and '[' in statement: 
            self.logs_pub.publish(String('    '+statement))
        if not override:
            if self.auto_execution:
                rospy.loginfo(statement + " Executing.")
                return True
        rospy.loginfo(statement + " Start Execution? (Y/N): ")
        command = input()
        check = str(command).lower().strip() if command else ''
        if check[0] == 'y':
            return True
        elif check[0] == 'n':
            return False
        elif check[0] == 'r':
            return None
        else:
            rospy.logwarn('Invalid Input!')
            return self.check_command(statement)

    def wait_for_completion_l(self):
        """ pause until left arm angle within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.TIME_TOLERANCE:
            if self.check_completion_l():
                return True
            else:
                rospy.sleep(0.01)
        rospy.logwarn("Action timed-out. Not able to complete in "+str(self.TIME_TOLERANCE)+'s')
        return False

    def wait_for_completion_r(self):
        """ pause until right arm angle within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.TIME_TOLERANCE:
            if self.check_completion_r():
                return True
            else:
                rospy.sleep(0.01)
        rospy.logwarn("Action timed-out. Not able to complete in "+str(self.TIME_TOLERANCE)+'s')
        return False

    def check_completion_l(self):
        """ check if left arm angles are within tolerance """
        error_list = [sqrt((s-t)**2) for s, t in zip(self.left_arm_state, self.left_arm_target)]
        error = sum(error_list) / len(error_list)
        return error<=self.POS_TOLERANCE

    def check_completion_r(self):
        """ check if right arm angles are within tolerance """
        error_list = [sqrt((s-t)**2) for s, t in zip(self.right_arm_state, self.right_arm_target)]
        error = sum(error_list) / len(error_list)
        return error<=self.POS_TOLERANCE
    
    def yumi_states_sb(self, msg):
        """ saves yumi states into this class """
        self.left_arm_state = msg.position[7:14]
        self.right_arm_state = msg.position[:7]
        self.left_gripper_state = msg.position[16]*1000
        self.right_gripper_state = msg.position[14]*1000

    def save_plan(self, plan, path):
        plan_file = open(path, 'w')
        yaml.dump(plan, plan_file, default_flow_style=True)
        plan_file.close()

    def execute_plan_from_file(self, path, arm=None):
        plan_file = open(path, 'r')
        plan = yaml.load(plan_file, Loader=yaml.Loader)
        plan_file.close()

        if arm=='right':
            self.right_arm_group.stop()
            self.right_arm_target = plan.joint_trajectory.points[-1].positions
            self.right_arm_group.execute(plan, wait=False)
            self.wait_for_completion_r()
        elif arm=='left':
            self.left_arm_group.stop()
            self.left_arm_target = plan.joint_trajectory.points[-1].positions
            self.left_arm_group.execute(plan, wait=False)
            self.wait_for_completion_l()

    @staticmethod
    def to_urdf_index(p):
        return [p[0], p[1], p[6], p[2], p[3], p[4], p[5]]

    @staticmethod
    def to_natural_index(p):
        return [p[0], p[1], p[3], p[4], p[5], p[6], p[2]]

    def setConstriants(self, group):
        """ Set constraint """
        moveit_constraints = Constraints()
        pcm = PositionConstraint()
        pcm.link_name = group.get_end_effector_link()
        pcm.header.frame_id = self.robot_frame
        pcm.target_point_offset = Point(0.0, 0.0, 0.0)
        pcm.weight = 1.0

        boundary = SolidPrimitive()
        boundary.type = boundary.BOX
        boundary.dimensions = [None]*3
        boundary.dimensions[boundary.BOX_X] = 1.2
        boundary.dimensions[boundary.BOX_Y] = 1.2
        boundary.dimensions[boundary.BOX_Z] = 1.0 # yumi height 0.571m
        pcm.constraint_region.primitives.append(boundary)
        boundary_pose = Pose()
        boundary_pose.position = Point(0,0,0)
        boundary_pose.orientation = Quaternion(0,0,0,1)
        pcm.constraint_region.primitive_poses.append(boundary_pose)
        moveit_constraints.position_constraints.append(pcm)

        # # Orientation constraint (keep the end effector at the same orientation)
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.link_name = group.get_end_effector_link()
        # orientation_constraint.orientation = Quaternion(*quaternion_from_euler(0.0, pi, pi))
        # moveit_constraints.orientation_constraints.append(orientation_constraint)
        group.set_path_constraints(moveit_constraints)

    def add_floor_plane(self, table_height=0.0):
        """ Add collision objects """
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = self.robot_frame
        plane_pose.pose.position = Point(x=0.4, y=0.0, z=table_height)
        plane_pose.pose.orientation = Quaternion(0,0,0,1)
        self.scene.add_box("floor", plane_pose, (1.0, 2.0, .01))

    def add_collision_box(self, name, position, size):
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = self.robot_frame
        plane_pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        plane_pose.pose.orientation = Quaternion(0,0,0,1)
        self.scene.add_box(name, plane_pose, tuple(size))

    def add_collision_mesh(self, name, position, quaternion, obj_path):
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = self.robot_frame
        plane_pose.pose.position = Point(*position)
        plane_pose.pose.orientation = Quaternion(*quaternion)
        self.scene.add_mesh(name, plane_pose, obj_path)

    def remove_collision_objects(self, name="floor"):
        self.scene.remove_world_object(name)

    def plot_marker(self, position, shape='arrow', id=0, orientation=[0,0,0,1]):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        if shape=='cube':
            marker.type = marker.CUBE
        elif shape=='sphere':
            marker.type = marker.SPHERE
        elif shape=='arrow':
            marker.type = marker.ARROW
        elif shape=='cylinder':
            marker.type = marker.CYLINDER
        else:
            rospy.logerr("Invalid marker shape! Got shape {}".format(shape))
            return
        marker.action = marker.ADD
        marker.pose.position = Point(*position)
        if len(orientation) == 4:
            marker.pose.orientation = Quaternion(*orientation)
        elif len(orientation) == 3:
            marker.pose.orientation = Quaternion(*quaternion_from_euler(*orientation))
        else:
            rospy.logerr("Invalid marker orientation! Got orientation {}".format(orientation))
            return
        marker.scale.x = .02
        marker.scale.y = .02
        marker.scale.z = .02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0) # 0 means forever
        self.marker_pub.publish(marker)


    ###############
    ### Gripper ###
    ###############

    def wait_for_completion_gripper_l(self, log=True):
        """ pause until left gripper state within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.GRIPPER_TIME_TOLERANCE:
            if self.check_completion_gripper_l():
                return True
            else:
                rospy.sleep(0.01)
        if log:
            rospy.logwarn("Gripper action timed-out. Not able to complete in "+str(self.GRIPPER_TIME_TOLERANCE)+'s')
        return False

    def wait_for_completion_gripper_r(self, log=True):
        """ pause until right gripper state within tolerance """
        time_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-time_start).to_sec()<self.GRIPPER_TIME_TOLERANCE:
            if self.check_completion_gripper_r():
                return True
            else:
                rospy.sleep(0.01)
        if log:
            rospy.logwarn("Gripper action timed-out. Not able to complete in "+str(self.GRIPPER_TIME_TOLERANCE)+'s')
        return False

    def check_completion_gripper_l(self):
        """ check if left gripper commands are within tolerance """
        error = sqrt((self.left_gripper_state-self.left_gripper_target)**2)
        return error<=self.GRIPPER_TOLERANCE

    def check_completion_gripper_r(self):
        """ check if right gripper commands are within tolerance """
        error = sqrt((self.right_gripper_state-self.right_gripper_target)**2)
        return error<=self.GRIPPER_TOLERANCE

    def closeRightGripper(self, wait=True, log=False):
        """ Close the right gripper """
        # cls.setRightGripperPos(0)
        self.sendRightGripperCmd(10, wait, log)

    def openRightGripper(self, wait=True, log=False):
        """ Open the right gripper """
        # cls.setRightGripperPos(25)
        self.sendRightGripperCmd(-10, wait, log)

    def closeLeftGripper(self, wait=True, log=False): 
        """ Close the left gripper """
        self.sendLeftGripperCmd(10, wait, log)
        # cls.setLeftGripperPos(0)

    def openLeftGripper(self, wait=True, log=False):
        """ Open the right gripper """
        # cls.setLeftGripperPos(25)
        self.sendLeftGripperCmd(-10, wait, log)

    def sendRightGripperCmd(self, cmd, wait=True, log=False):
        """ Send an effort command to the right gripper """
        noise = random()/100
        msg = Float64()
        msg.data = cmd+noise
        self.right_gripper_target = 25 if cmd<0 else 0
        self.right_gripper_cmd_pub.publish(msg)
        if wait and cmd!=0:
            self.wait_for_completion_gripper_r(log=log)

    def sendLeftGripperCmd(self, cmd, wait=True, log=False):
        """ Send an effort command to the left gripper """
        noise = random()/100
        msg = Float64()
        msg.data = cmd+noise
        self.left_gripper_target = 25 if cmd<0 else 0
        self.left_gripper_cmd_pub.publish(msg)
        if wait and cmd!=0:
            self.wait_for_completion_gripper_l(log=log)

    def setRightGripperPos(self, pos, wait=True, log=False): # pos in mm, opening of one side
        """ Send a position command to the right gripper """
        noise = random()/100
        msg = Float64()
        msg.data = self.right_gripper_target = pos+noise
        # msg.data = self.right_gripper_target
        self.right_gripper_pos_pub.publish(msg)
        if wait:
            self.wait_for_completion_gripper_r(log=log)

    def setLeftGripperPos(self, pos, wait=True, log=False):
        """ Send a position command to the left gripper """
        noise = random()/100
        msg = Float64()
        msg.data = self.left_gripper_target = pos+noise
        # msg.data = pos+noise
        self.left_gripper_pos_pub.publish(msg)
        if wait:
            self.wait_for_completion_gripper_l(log=log)

if __name__ == "__main__":
    rospy.init_node('yumi_ctrl_node', anonymous=True)
    yumi = YumiCtrl(auto_execution=True)
    yumi.left_go_to_named('grasp', wait=False)
    rospy.sleep(1)
    yumi.right_go_to_named('grasp')
    # print(yumi.left_arm_group.get_current_pose())
    # yumi.left_go_thro([[0.45, 0.2, 0.23, 0, pi, 0]],"test")
    # yumi.right_go_to_named('grasp')
    # print(yumi.right_arm_group.get_current_pose())
    # yumi.right_go_thro([[0.45, -0.2, 0.23, 0, pi, 0]],"test")
