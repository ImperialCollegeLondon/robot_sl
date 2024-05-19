#!/usr/bin/python3

import json
from re import findall
import sys
import signal
import numpy as np
from time import time

import rospy
np.float = np.float64
from ros_numpy import msgify
from sl_msgs.srv import *
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16, Int8, Int8MultiArray, Float32MultiArray

from sl_ctrl import ShoelacingPrimitives

class ShoelacingControlNode:
    pattern_srv = 'find_pattern'
    plan_srv = 'find_plan'
    def __init__(self):
        # read parameters
        auto_execution = True
        reset = True
        self.sim = rospy.get_param("~sim", False)

        self.action_pub = rospy.Publisher("/sl_ctrl/action", String, queue_size=10)
        self.progress_pub = rospy.Publisher("/sl_ctrl/progress", Int16, queue_size=10)
        self.find_pattern = rospy.ServiceProxy(self.pattern_srv, findPatternService)
        self.find_plan = rospy.ServiceProxy(self.plan_srv, findPlanService)
        if self.sim:
            from uni_lace_msgs.srv import UniLaceInfoService, UniLaceInfoServiceRequest
            # register info service client
            self.sim_metric_client = rospy.ServiceProxy('unity_metric_info', UniLaceInfoService)

        self.action_pub.publish(String('Initialising YuMi ...'))
        self.sl_ctrl = ShoelacingPrimitives(auto_execution, reset, sim=self.sim)

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        '''Handles KeyboardInterrupts to ensure smooth exit'''
        rospy.logwarn('User Keyboard interrupt')
        self.stop()
        sys.exit(0)
    
    def stop(self):
        self.sl_ctrl.stop()

    def calc_pattern_length(self, pattern):
        '''Calculate the length of the pattern'''
        length = 0
        for i in range(len(pattern)-1):
            length += self.sl_ctrl.pm.eyelet_distances[pattern[i], pattern[i+1]]
        return length

    def run(self):
        start_time = time()
        self.sl_ctrl.add_to_log('[Start time] '+ str(start_time))
        prev_a = 0
        prev_b = 0
        # design a shoe lacing pattern
        pattern_start_time = time()
        self.action_pub.publish(String('Designing Shoe Lacing Pattern'))
        request = findPatternServiceRequest()
        request.num_eyelets = Int8(self.sl_ctrl.pm.n_rows*2)
        request.eyelet_positions = Float32MultiArray()
        request.eyelet_positions.data = np.array(self.sl_ctrl.pm.eyelet_poses)[:,:3].flatten().tolist()
        response = self.find_pattern(request)
        pattern = response.pattern.data
        current_time = time()
        self.sl_ctrl.add_to_log('[Pattern found] Time cost: {}. Current time: \
                                {}'.format(current_time-pattern_start_time, current_time))
        self.sl_ctrl.add_to_log('[Pattern] '+ str(pattern))
        self.sl_ctrl.add_to_log('[Pattern Length] {}'.format(self.calc_pattern_length(pattern)))
        self.action_pub.publish(String('Pattern Found \nin {:.2f}s\
                                       '.format(current_time-pattern_start_time)))
        self.progress_pub.publish(Int16(0))

        # send the pattern to planner
        plan_start_time = time()
        self.action_pub.publish(String('Planning For Shoe Lacing'))
        request = findPlanServiceRequest()
        request.pattern = Int8MultiArray()
        request.pattern.data = pattern
        request.aesthetic_mode.data = 1 # 1 top-right first 2 top-left first
        response = self.find_plan(request)
        actions = response.plan.data
        current_time = time()
        self.action_pub.publish(String('Plan Found \nin {:.2f}s\
                                       '.format(current_time-plan_start_time)))
        self.sl_ctrl.add_to_log('[Plan found] Time cost: {}. Current time: \
                                {}'.format(current_time-plan_start_time, current_time))
        self.sl_ctrl.add_to_log('[Plan] '+ str(actions))

        # parse the plan
        actions = findall('\(.*?\)',actions)
        action_list = []
        for a in actions:
            a_list = findall('\(.*?\)',a)[0][1:-1].split()
            action_list.append(a_list)

        # start execution
        for i, a in enumerate(action_list):
            text = '\n'.join(a[:2])
            self.action_pub.publish(String(text))
            sl_cost = 0
            if 'insert_a' in a[0]:
                next = int(a[1][6:])
                sl_cost = self.sl_ctrl.pm.eyelet_distances[prev_a, next]
                prev_a = next
            elif 'insert_b' in a[0]:
                next = int(a[1][6:])
                sl_cost = self.sl_ctrl.pm.eyelet_distances[prev_b, next] if prev_b!=0 else 0
                prev_b = next

            print('sl_cost:{}'.format(sl_cost))
            print('Current action: {}'.format(a))
            self.sl_ctrl.execute(a, action_list[i+1] if i<len(action_list)-1 else None, sl_cost=sl_cost)
            # update progress
            progress = int((i+1)*100.0/len(action_list))
            self.progress_pub.publish(Int16(progress))
            current_time = time()
            self.sl_ctrl.add_to_log('[Action finished] Time cost: {}. Current time: \
                                    {}'.format(current_time-start_time, current_time))
        rospy.loginfo("Mission accomplished in {}".format(str(current_time-start_time)))
        # fetch metrics info from simulation
        if self.sim:
            response = self.sim_metric_client()
            info = json.loads(response.info.data)
            print(info)
            self.sl_ctrl.add_to_log('[Simulation Metrics] '+ str(info))

if __name__ == "__main__":
    rospy.init_node('sl_ctrl_node', anonymous=True)

    # Run the sequence of actions
    sl_ctrl_node = ShoelacingControlNode()
    sl_ctrl_node.run()