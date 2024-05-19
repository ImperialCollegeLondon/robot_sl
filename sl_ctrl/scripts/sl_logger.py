#!/usr/bin/python3

from datetime import datetime
from os import path

import rospkg
import rospy
from sl_msgs.srv import *
from std_msgs.msg import String


class ShoelacingLogger:
    package_name = 'sl_ctrl'
    log_topic = 'sl_logs'
    def __init__(self):
        self.log_path = path.join(rospkg.RosPack().get_path(self.package_name), 'results', 'logs.txt')
        self.log_file = open(self.log_path, 'w+')
        self.log_file.close()
        rospy.Subscriber(self.log_topic, String, self.log_cb, queue_size=10)
        rospy.spin()

    def add_to_log(self, content):
        self.log_file = open(self.log_path, 'a')
        self.log_file.write('[{}] {}'.format(datetime.now(), content) +'\n')
        self.log_file.close()
    
    def log_cb(self, msg):
        self.add_to_log(msg.data)


if __name__ == "__main__":
    rospy.init_node('sl_logger', anonymous=True)
    logger = ShoelacingLogger()