#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('learning_tf')
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from dynamic_reconfigure.server import Server
from marker_based_localisation.cfg import PoseBiasAdjust

class Bias_corrector:
    
    def __init__(self):
        #get private NS params from bash or launch file
        self.init_message = rospy.get_param('~start message','Adjust Node started')
        self.input_topic = rospy.get_param('~input_topic','/vrpn_client_node/Solo/pose')
        self.output_topic = rospy.get_param('~output_topic','/ground_truth/pose')
        rospy.loginfo('Input_topic: %s', self.input_topic)
        rospy.loginfo('Output_topic: %s', self.output_topic)

        #Start dynamic reconfigure server
        self.server = Server(PoseBiasAdjust,self.reconfigure)

        self.sub = rospy.Subscriber(self.input_topic,Pose,self.correct_bias)
        self.pub = rospy.Publisher(self.output_topic,Pose,queue_size = 10)
        rospy.spin()

    def correct_bias(self,data):

        
        #construct a new message from Imu data Class and add header
        msg = Pose(header=Header(stamp=rospy.get_rostime()))
        
        # adjust bad data by subracting bias 
        good_x = bad_data.linear_acceleration.x-self.first_msg.linear_acceleration.x
        good_y = bad_data.linear_acceleration.y-self.first_msg.linear_acceleration.y
        good_z = bad_data.linear_acceleration.z-self.first_msg.linear_acceleration.z
        
        # rewrite new topic
        msg.angular_velocity = bad_data.angular_velocity
        msg.angular_velocity_covariance = bad_data.angular_velocity_covariance
        msg.orientation = bad_data.orientation
        msg.orientation_covariance = bad_data.orientation_covariance
        msg.linear_acceleration_covariance = bad_data.linear_acceleration_covariance
        msg.linear_acceleration.x = good_x
        msg.linear_acceleration.y = good_y
        msg.linear_acceleration.z = good_z
        
        self.pub_good_imu.publish(msg)
        rospy.loginfo('Bias corrected IMU linear acc')
        
    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.message = config["message"]
        self.a = config["a"]
        self.b = config["b"]
        # Return the new variables.
        return config

if __name__ == '__main__':
    try:
        rospy.init_node('pose_adjuster')
        bc = Bias_corrector()
    except rospy.ROSInterruptException:
        pass