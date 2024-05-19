#!/usr/bin/python3

import rospy
from sl_visn_colored import shoelacingVision

class shoelacingVisionNode:
    auto_execution = True
    def __init__(self):
        # read parameters
        self.sim = rospy.get_param("~sim", False)
        self.sl_visn = shoelacingVision(self.auto_execution, self.sim)    

if __name__ == "__main__":
    rospy.init_node('sl_visn_node', anonymous=True)
    sl_visn_node = shoelacingVisionNode()
    rospy.spin()
