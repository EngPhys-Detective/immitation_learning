#!/usr/bin/env python3

from __future__ import print_function
 

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tensorflow import keras as ks

cnn_path = '/home/fizzer/enph_ws/src/immitation_learning/models/cnn1.h5'

class Driver():
    
    startmsg = "TeamName,password,0,NA"
    endmsg = "TeamName,password,-1,NA"

    initial_position = [5.5, 2.5]
     
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    def __init__(self, run_count, cnn_path) -> None:
        self.bridge = CvBridge()
        
        rospy.init_node('keyboard_controller')
        # self.score_pub = rospy.Publisher('score_tracker', String , queue_size=1)
        self.velocity_pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
        self.postition = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback)
        self.count = 0
        self.run_count = run_count
        self.prev_key = None
        self.twist = Twist()
        self.cnn_model = ks.models.load_model(cnn_path)
        print("----- CNN Model Successfully Loaded -----")

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

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.move(self.predict_command(cv_image))
    
    
    def move(self, command):
        if command == 0:
            self.twist.linear.x = 0.75
            self.twist.angular.z = 0
        elif command == 1:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
        elif command == 2:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.5
        elif command == 3:
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        self.velocity_pub.publish(self.twist)
        rospy.sleep(0.1)
        self.velocity_pub.publish(self.stop_msg)

    def predict_command(self, cv_image):
        # resize image to 72x128
        cv_image = cv2.resize(cv_image, (128, 72))
        # add axis
        prediction = self.cnn_model.predict(np.expand_dims(cv_image, axis=0))
        return np.argmax(prediction)

if __name__ == '__main__': 
    driver = Driver(5, cnn_path)
    print(driver.startmsg)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
