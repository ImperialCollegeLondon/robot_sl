#!/usr/bin/python3

import subprocess
import time
import numpy as np
from os import path, walk
from re import findall

import rospy
from std_msgs.msg import Int8, Int8MultiArray
from sl_msgs.srv import *


DEBUG=False

class ShoelacingPlanner:
    '''
        This class is used to plan for a sequece of actions to lace a shoe.
        The planner is based on LAMA planner (http://www.fast-downward.org/)
        Origonal planner uses implementation with Planutils (https://github.com/HainingLuo/planutils)

        This class is implemented as a ROS service server, 
        it expects a request with:
        pattern: 
            A list of integers, representing the connection between eyelets
        aesthetic_mode: 
            0: no specification
            1: leftward on top
            2: rightward on top
    '''
    find_plan_srv = 'find_plan'
    def __init__(self):
        rospy.init_node('sl_planner', anonymous=True)

        # start the ROS service
        rospy.Service(self.find_plan_srv, findPlanService, self.plan)
        rospy.loginfo('Find pattern service ready')

    def init_prob(self, num_rows):
        self.prob_name = 'shoe_lacing'
        self.domain_name = 'shoe'
        self.file_path = path.join(path.dirname(path.realpath(__file__)), 'pddl')

        self.objects = {}
        self.num_aglets = 2
        self.objects['aglet'] = ['aglet_a', 'aglet_b']
        self.num_rows = num_rows
        self.num_eyelets = 2*num_rows
        self.objects['eyelet_left'] = ['eyelet{}'.format(i*2) for i in range(self.num_rows)]
        self.objects['eyelet_right'] = ['eyelet{}'.format(i*2+1) for i in range(self.num_rows)]
        self.objects['eyelet'] = ['eyelet{}'.format(i) for i in range(self.num_rows*2)]
        self.objects_list = ['eyelet_left', 'eyelet_right']

        # States
        self.states = {}
        self.states['initial_site'] = ['site_l1', 'site_r1']
        self.states['cursor_a'] = 'eyelet0'
        self.states['cursor_b'] = 'eyelet1'

        # Goals
        self.goals = {}
        self.goals['eyelet_laced'] = ['eyelet{}'.format(i) for i in range(self.num_rows*2)]
        self.pattern = None

    def set_pattern(self, pattern):
        # cut at the last 2 eyelets
        id1 = np.where(pattern==np.max(pattern))[0][0]
        id2 = np.where(pattern==np.max(pattern)-1)[0][0]
        if abs(id1-id2)==1:
            incision = np.max([id1, id2])
            pattern = np.concatenate((pattern[incision:], pattern[:incision]))
        pattern = np.array(pattern)
        # cut at the first eyelet
        id0 = np.where(pattern==0)[0][0]
        sequence0 = pattern[:id0] if pattern[0]<=pattern[id0-1] else np.flip(pattern[:id0])
        sequence1 = pattern[id0:] if pattern[id0]<=pattern[-1] else np.flip(pattern[id0:])
        pattern = [np.array(sequence0).tolist(), np.array(sequence1).tolist()]
        # reposition the 0
        for p in pattern:
            if 0 in p and 1 in p: p.remove(0)
            if 0 not in p and 1 not in p: p.insert(0,0)
        if 0 in pattern[1]:
            self.pattern = pattern
        else:
            self.pattern = [pattern[1],pattern[0]]
        
    def gen_problem(self):
        # build the problem file
        define = self.gen_prob_define(self.prob_name, self.domain_name)
        objects = self.gen_prob_objects()
        init = self.gen_prob_init()
        goal = self.gen_prob_goal()
        problem = '(' + define + objects + init + goal + ')'
        return problem

    def gen_fake_problem(self):
        f = open(path.join(self.file_path, "problem.pddl"), "r")
        problem = f.read()
        f.close()
        return problem

    def gen_prob_define(self, prob_name, domain_name):
        return 'define (problem {}) (:domain {}) \n'.format(prob_name, domain_name)

    def gen_prob_objects(self):
        objs_str = ''
        for obj in self.objects_list:
            objs_str+='\n\t'
            for ins in self.objects[obj]: objs_str+=ins+' '
            objs_str+='- '+ obj
        return "(:objects{}\n)\n".format(objs_str)

    def gen_prob_init(self):
        # initial states
        aglet_at = ''
        at_correct_side = ''
        correct_side = ''
        for a,s in zip(self.objects['aglet'], self.states['initial_site']):
            aglet_at+='\n\t(aglet_at {} {})'.format(a, s)
            at_correct_side += '\n\t(at_correct_side {})'.format(a)
            if 'l' in s:
                correct_side += '\n\t(left_correct_side {})'.format(a)
            elif 'r' in s:
                correct_side += '\n\t(right_correct_side {})'.format(a)

        occupied = ''
        for site in self.states['initial_site']:
            occupied += '\n\t(occupied {})'.format(site)

        # pattern requirements
        lace_with = ''
        for e in self.pattern[1]:
            lace_with+='\n\t(lace_with eyelet{} {})'.format(e, self.objects['aglet'][0])
        for e in self.pattern[0]:
            lace_with+='\n\t(lace_with eyelet{} {})'.format(e, self.objects['aglet'][1])

        block_pairs_i = []
        # insertion requirements
        for i in range(self.num_eyelets-1, 1, -1):
            for j in range(i-i%2):
                block_pairs_i.append([i, j])

        block_pairs_p = []
        block_pairs_p_set = []
        # pattern requirements (same row, lace in sequence)
        for strand in self.pattern:
            for i in range(1, len(strand)-1, 1):
                if strand[i]//2 == strand[i+1]//2:
                    block_pairs_p.append([strand[i+1], strand[i]])
                    block_pairs_p_set.append(set([strand[i+1], strand[i]]))

        # aesthetic requirements
        block_pairs_a = []
        if len(block_pairs_p)!=0:
            # override aesthretic mode
            self.aesthetic_mode = 1 if block_pairs_p[0][0]>block_pairs_p[0][1] else 2
        for i in range(0, self.num_eyelets, 2):
            pair_set = set([i+1, i])
            if pair_set in block_pairs_p_set:
                continue
            if self.aesthetic_mode==1:
                block_pairs_a.append([i+1, i])
            elif self.aesthetic_mode==2:
                block_pairs_a.append([i, i+1])

        block_pairs = np.array(block_pairs_i+block_pairs_p+block_pairs_a)
        _, indeces = np.unique(block_pairs, axis=0, return_index=True)
        block_pairs = block_pairs[sorted(indeces)]

        # write block predicates
        block = ''
        for p in block_pairs:
            block+='\n\t(block eyelet{} eyelet{})'.format(p[0], p[1])

        return "(:init{}{}{}{}{}{}\n)\n".format(aglet_at, occupied, at_correct_side, correct_side, lace_with, block)

    def gen_prob_goal(self):
        eyelet_laced = ''
        for e in self.goals['eyelet_laced']:
            eyelet_laced+='\n\t(eyelet_laced {})'.format(e)
        return """(:goal (and{}\n))\n""".format(eyelet_laced)

    def gen_prob_minimise(self, criteria):
        return "(:metric minimize ({}))".format(criteria)

    def plan(self, request):
        pattern = request.pattern.data
        self.aesthetic_mode = request.aesthetic_mode.data
        self.init_prob(len(pattern)//2)
        self.set_pattern(pattern)
        # generate question file
        problem = self.gen_problem()

        # run the solver
        print('Planning for {}'.format(self.prob_name))
        start = time.time()

        # save the contents to files
        f = open(path.join(self.file_path, "problem.pddl"), "w")
        f.write(problem)
        f.close()
        result = subprocess.run(["/software/downward/fast-downward.py", \
                                "--alias", "lama", \
                                "--plan-file", path.join(self.file_path, "plan.txt"), \
                                path.join(self.file_path, "domain.pddl"), \
                                path.join(self.file_path, "problem.pddl")], 
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout.decode()
        # print(result)
        fns = []
        plans = ''
        for root, dirs, files in walk(self.file_path):
            for file in files:
                if file.startswith("plan.txt"):
                    fns.append(path.join(root, file))
        for fn in fns:
            f = open(fn, "r")
            plans+=f.read()+'\n'    
        # process the results 
        plan = self.decode_plan(plans)
        # delete the plan files
        for fn in fns:
            subprocess.run(["rm", fn])
        end = time.time()
        response = findPlanServiceResponse()
        print('Solution found in {:.2f}s'.format(end-start))
        response.plan.data = plan
        return response

    def decode_plan(self, plan_text):
        plan = plan_text.split(sep='\n')
        costs = []
        bookmarks = [0]
        for id, action in enumerate(plan):
            if 'cost' in action:
                cost = action.split()[3]
                costs.append(int(cost))
                bookmarks.append(id+2)
        if len(costs)!=0:
            best_plan_id = np.argmin(costs)
            best_plan = plan[bookmarks[best_plan_id]:bookmarks[best_plan_id+1]-2]
            print('Best cost'.format(costs[best_plan_id]))
            return '\n'.join(best_plan)
        else:
            print('No valid plan found')
            return ''

if __name__ == '__main__':
    planner = ShoelacingPlanner()

    if not DEBUG:
        rospy.spin()
    else:
        # pattern = [0, 3, 4, 5, 2, 1]
        # pattern = [0, 5, 4, 3, 2, 1]
        pattern = [0, 3, 2, 5, 4, 1]
        # pattern = [0,3,4,7,8,9,6,5,2,1]
        # pattern = [8, 6, 7, 5, 2, 0, 1, 3, 4, 9]
        request = findPlanServiceRequest()
        request.pattern = Int8MultiArray()
        request.pattern.data = pattern
        request.aesthetic_mode = Int8(2)
        response = planner.plan(request)
        actions = response.plan.data
        actions = findall('\(.*?\)',actions)
        action_list = []
        for a in actions:
            a_list = findall('\(.*?\)',a)[0][1:-1].split()
            action_list.append(a_list)
        print(action_list)
        print(len(action_list))