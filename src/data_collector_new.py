#!/usr/bin/env python3

from __future__ import print_function
 
import roslib
import rospy
import cv2
import time
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class KeyboardController():
    
    startmsg = "TeamName,password,0,NA"
    endmsg = "TeamName,password,-1,NA"

    initial_position = [5.5, 2.5]
    prev_key = None
    count = 0
 
    target_path = '/home/fizzer/enph_ws/src/immitation_learning/data'
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    def __init__(self, run_count) -> None:
        self.bridge = CvBridge()
        
        rospy.init_node('keyboard_controller')
        # self.score_pub = rospy.Publisher('score_tracker', String , queue_size=1)
        self.velocity_pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
        self.postition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback)
        self.run_count = run_count

        # self.reset_position()

    def reset_position(self):
        model_state = ModelState()
        model_state.model_name = "R1"
        model_state.pose.position.x = self.initial_position[0]
        model_state.pose.position.y = self.initial_position[1]
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = -1.5690
        model_state.pose.orientation.w = 0.0
        self.postition(model_state)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == 'q':
                print(self.endmsg)
                sys.exit()
        else:
            key = None
        return key
    
    def move(self, key):
        rate = rospy.Rate(50)
        twist = Twist()
        if key == 'i':
            twist.linear.x = 0.4
            twist.angular.z = 0
        elif key == 'j':
            twist.linear.x = 0
            twist.angular.z = 0.5
        elif key == 'l':
            twist.linear.x = 0
            twist.angular.z = -0.5
        elif key == 'k':
            twist.linear.x = 0
            twist.angular.z = 0

        self.velocity_pub.publish(twist)
        rate.sleep()
        # pause for 0.5 seconds
        time.sleep(0.5)
        self.velocity_pub.publish(self.stop_msg)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        key = self.get_key()
        self.move(key)
        self.prev_key = key

        if key != None:    
            self.save_image(key, cv_image)
            self.count += 1
    
    def save_image(self, key, cv_image):
        if key == 'i':
            cv2.imwrite(self.target_path + '/forward/' + str(self.run_count) + '_' + str(self.count) + '.png', cv_image)
        elif key == 'j':
            cv2.imwrite(self.target_path + '/left/' + str(self.run_count) + '_' + str(self.count) + '.png', cv_image)
        elif key == 'l':
            cv2.imwrite(self.target_path + '/right/' + str(self.run_count) + '_' + str(self.count) + '.png', cv_image)
        elif key == 'k':
            cv2.imwrite(self.target_path + '/stop/' + str(self.run_count) + '_' + str(self.count) + '.png', cv_image)

if __name__ == '__main__': 
    kc = KeyboardController(1)
    print(kc.startmsg)
    kc.move(kc.get_key())

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
