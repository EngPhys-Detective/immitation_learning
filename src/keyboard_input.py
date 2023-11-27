#!/usr/bin/env python3

from __future__ import print_function
 
import roslib
import sys
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
 
    target_path = '/home/fizzer/enph_ws/src/immitation_learning/data'

    def __init__(self) -> None:
        self.bridge = CvBridge()
        
        rospy.init_node('keyboard_controller')
        # self.score_pub = rospy.Publisher('score_tracker', String , queue_size=1)
        self.velocity_pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
        self.postition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback)

        self.reset_position()
        # self.data_init()

    def data_init(self):
        if not os.path.exists(os.path.join(self.target_path, 'forward')):
            os.mkdir(self.target_path + '/forward')    
        if not os.path.exists(os.path.join(self.target_path, 'left')):
            os.mkdir(self.target_path + '/left')
        if not os.path.exists(os.path.join(self.target_path, 'right')):
            os.mkdir(self.target_path + '/right')
        if not os.path.exists(os.path.join(self.target_path, 'stop')):
            os.mkdir(self.target_path + '/stop')

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
        else:
            key = self.prev_key
        return key

    def keyboard_control(self, key):
        # rate = rospy.Rate(50)
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0

        if key == 'i':
            move.linear.x = 0.25
            move.angular.z = 0.0
        elif key == 'j':
            move.linear.x = 0.0
            move.angular.z = 0.5
        elif key == 'l':
            move.linear.x = 0.0
            move.angular.z = -0.5
        elif key == 'q':
            sys.exit()
        elif key == 'k':
            move.linear.x = 0.0
            move.angular.z = 0.0

        self.velocity_pub.publish(move)
        # rate.sleep(1)

    def callback(self, data):

        try:
            camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # make image into numpy array
        camera_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2RGB)

        key = self.get_key()
        self.keyboard_control(key)

        self.prev_key = key
        
        cv2.imshow("image", camera_image)
        # self.save_image(camera_image, key)



    def save_image(self, image, key):
        if key == 'i':
            cv2.imwrit(self.target_path + '/forward/', str(time.time) + '.png', image)
        elif key == 'j':
            cv2.imwrite(self.target_path + '/left/', str(time.time) + '.png', image)
        elif key == 'l':
            cv2.imwrite(self.target_path + '/right/', str(time.time) + '.png', image)    
        elif key == 'k':
            cv2.imwrite(self.target_path + '/stop/', str(time.time) + '.png', image)
        

def main(args):
    
    tt = KeyboardController()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")    
    

if __name__ == '__main__':
    main(sys.argv)
