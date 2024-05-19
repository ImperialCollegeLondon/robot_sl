#!/usr/bin/python3

from os import path
import cv2
import numpy as np
import time
from math import sqrt
from yaml import safe_load
from scipy import optimize
from scipy.spatial import distance

import rospy
from sl_msgs.srv import *
from std_msgs.msg import Int8MultiArray

from genetic_algorithm import GeneticAlgorithm

class ShoelacingDesigner:
    '''
        This class is used to design the optimal pattern for shoelacing.
        It is based on the Genetic Algorithm (implemented in genetic_algorithm.py).
        Original implementation uses pymoo (https://pymoo.org/).

        This class is implemented to be used as a ROS service.
        The service is defined in sl_msgs/srv/findPatternService.srv.
        The service takes the following inputs:
            - num_eyelets: the number of eyelets in the shoe
            - shoelace_length: the length of the shoelace
            - eyelet_positions: the positions of the eyelets in the shoe
        The service returns the following outputs:
            - pattern: the optimal pattern for shoelacing
    '''
    
    find_pattern_srv = 'find_pattern'
    def __init__(self, n_rows, H=5.5, V=2.0, shoelace_length=137.0, eyelet_positions=None, gamma=0):
                
        self.n_rows = n_rows
        self.n_eyelets = n_rows*2
        self.shoelace_length = shoelace_length
        self.gamma = gamma
        self.bump_compensation = 0.03

        # compute the length of each segment
        self.segment_length = np.zeros((self.n_eyelets, self.n_eyelets))
        self.segment_tension = np.zeros((self.n_eyelets, self.n_eyelets))
        self.segment_length_h = np.zeros((self.n_eyelets, self.n_eyelets))

        if eyelet_positions is not None:
            self.eyelet_positions = np.array(eyelet_positions).reshape((self.n_eyelets, 3))
            for id1 in range(self.n_eyelets):
                for id2 in range(id1+1, self.n_eyelets):
                    segment_length = distance.euclidean(self.eyelet_positions[id1], self.eyelet_positions[id2])+self.bump_compensation
                    # compute horizontal distance
                    h_dist = abs(self.eyelet_positions[id1, 1]-self.eyelet_positions[id2, 1]) # TODO different in y
                    # compute the segment length and tension
                    segment_tension = h_dist/segment_length
                    self.segment_length[id1, id2] = self.segment_length[id2, id1] = segment_length
                    self.segment_tension[id1, id2] = self.segment_tension[id2, id1] = segment_tension
        else:
            for id1 in range(self.n_eyelets):
                for id2 in range(id1+1, self.n_eyelets):
                    # compute vertical distance
                    id1_row = id1//2
                    id2_row = id2//2
                    v_dist = abs(id1_row-id2_row)
                    # compute horizontal distance
                    id1_col = id1%2
                    id2_col = id2%2
                    h_dist = abs(id1_col-id2_col)
                    # compute the segment length and tension
                    segment_length = sqrt((v_dist*V)**2 + (h_dist*H)**2)
                    segment_tension = (h_dist*H)/segment_length
                    self.segment_length[id1, id2] = self.segment_length[id2, id1] = segment_length
                    self.segment_tension[id1, id2] = self.segment_tension[id2, id1] = segment_tension
                    self.segment_length_h[id1, id2] = self.segment_length_h[id2, id1] = h_dist*H

        self.max_total_length = self.calc_shortest()
        print(self.max_total_length)

    def calc_shortest(self):
        total_length = 0
        total_length+=self.segment_length[0, 1]
        for i in range(self.n_rows-1):
            total_length+=self.segment_length[2*i, 2*i+2]
            total_length+=self.segment_length[2*i+1, 2*i+3]
        return total_length

    def objective(self, idx):
        temp = sorted(idx)    
        idx = [temp.index(i) for i in idx]

        total_length = 0
        total_tension = 0
        total_length_h = 0
        for i in range(self.n_eyelets):
            id1 = idx[i-1]
            id2 = idx[i]
            section_length = self.segment_length[id1, id2]
            section_tension = self.segment_tension[id1, id2]
            section_length_h = self.segment_length_h[id1, id2]
            total_length+=section_length
            total_tension+=section_tension
            total_length_h+=section_length_h
            quality = total_length*self.gamma-total_tension*(1-self.gamma)
            # quality = total_length*self.gamma/self.max_total_length-total_length_h/total_length*(1-self.gamma)
        return quality + 10 # keep it positive
    
    @staticmethod
    def constr_definition(x): # the adjacent 3 must be from different columns
        for i in range(len(x)):
            if x[i-2]%2==x[i-1]%2==x[i]%2:
                return 1 # return true if any of x_i holds
        return 0

    @staticmethod
    def constr_simple(x):
        max_id = np.argmax(x)
        ascend = all([x[i-1]//2<=x[i]//2 for i in range(1, max_id)])
        descend = all([x[i-1]//2>=x[i]//2 for i in range(max_id+1, len(x))])
        # straight ascend and then descend (rely on constraint 1)
        if ascend and descend:
            return 0
        else:
            return 1

    @staticmethod
    def constr_straight(x):
        for i in range(len(x)):
            if x[i-2]//2!=x[i-1]//2 and x[i-1]//2!=x[i]//2:
                return 1 # return true if any of x_i holds
        return 0

    @staticmethod
    def constr_dense(x):
        for i in range(len(x)):
            if x[i-1]%2==x[i]%2:
                return 1 # return true if any of x_i holds 
        return 0

    def design(self, dense=False, straight=False, simple=False):
        # start the optimisation
        print('Finding the best pattern.')
        start = time.time()
        constraints = [self.constr_definition]
        if dense: constraints.append(self.constr_dense)
        if straight: constraints.append(self.constr_straight)
        if simple: constraints.append(self.constr_simple)
        ga = GeneticAlgorithm(self.objective, len_x=self.n_eyelets, constraints=constraints)
        res = ga.solve()[0]
        end = time.time()
        print('Best pattern found in {:.2f}s.'.format(end-start))
        return self.adjust_pattern(res)
    
    def adjust_pattern(self, pattern):
        # cut at the last 2 eyelets
        id1 = np.where(pattern==np.max(pattern))[0][0]
        id2 = np.where(pattern==np.max(pattern)-1)[0][0]
        if abs(id1-id2)==1:
            incision = np.max([id1, id2])
            pattern = np.concatenate((pattern[incision:], pattern[:incision]))
        pattern = np.array(pattern)
        return pattern

class ShoelacingDesignerService:
    find_pattern_srv = 'find_pattern'
    def __init__(self):
        rospy.init_node('sl_designer', anonymous=True)
        rospy.Service(self.find_pattern_srv, findPatternService, self.design_cb)
        rospy.loginfo('Find pattern service ready')
        rospy.spin()

    def design_cb(self, request):
        gamma = 0
        straight = False
        dense = True
        designer = ShoelacingDesigner(
            request.num_eyelets.data//2, 
            shoelace_length=request.shoelace_length.data,
            eyelet_positions=request.eyelet_positions.data,
            gamma=gamma)
        idx = designer.design(dense=dense, straight=straight, simple=False)
        # construct the response
        response = findPatternServiceResponse()
        response.pattern = Int8MultiArray()
        response.pattern.data = idx
        return response

if __name__ == '__main__':
    ShoelacingDesignerService()