#!/usr/bin/env python

import rospy
import math 
from robp_msgs.msg import CommandDuration
from hiwonder_servo_msgs.msg import JointState
from std_msgs.msg import Float64


class RoboticArm:

    def __init__(self):
       
       
        # Define the lengths of the three links of the robotic arm
        self.length1 = 101.0
        self.length2 = 95.0
        self.length3 = 165.0

        # We initialize the orientation of the end effector 
        self.orientation = 0.0 

        # Velocity of the servos
        self.time_factor=(2500)/((math.pi)/2)

        # Initialize the joint angles to 0
        self.joint1_angle = 0.0
        self.joint2_angle = 0.0
        self.joint3_angle = 0.0
        self.joint4_angle = 0.0

        #initialize the current states of the joints to 0
        self.joint1_state = 0.0
        self.joint2_state = 0.0
        self.joint3_state = 0.0
        self.joint4_state = 0.0

        # Initialize the command duration to zero 
        self.command1_duration = 0.0
        self.command2_duration = 0.0
        self.command3_duration = 0.0
        self.command4_duration = 0.0

        # Define the publishers for the command of each joint
        self.joint1_command_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size=10)
        self.joint2_command_pub = rospy.Publisher('/joint2_controller/command_duration', CommandDuration, queue_size=10)
        self.joint3_command_pub = rospy.Publisher('/joint3_controller/command_duration', CommandDuration, queue_size=10)
        self.joint4_command_pub = rospy.Publisher('/joint4_controller/command_duration', CommandDuration, queue_size=10)
        self.gripper_pub = rospy.Publisher('/r_joint_controller/command', Float64, queue_size=10)
        
        # We Define the subscriber to get the joint states.
        self.state1_subs = rospy.Subscriber('/joint2_controller/state',JointState, self.state1_callback)
        self.state2_subs = rospy.Subscriber('/joint2_controller/state',JointState, self.state2_callback)
        self.state3_subs = rospy.Subscriber('/joint3_controller/state',JointState, self.state3_callback)
        self.state4_subs = rospy.Subscriber('/joint4_controller/state',JointState, self.state4_callback)
    
    def state1_callback(self,msg):
        self.joint1_state = msg.current_pos

    def state2_callback(self,msg):
        self.joint2_state = msg.current_pos
    
    def state3_callback(self,msg):
        self.joint3_state = msg.current_pos
            
    def state4_callback(self,msg):
        self.joint4_state = msg.current_pos
        
     
    
    def calculate_commands(self, y , z):
        self.busy = True
 
        # Set different orientations of the gripper according to the distance to the object.
        if y>=300.0:
            self.orientation = -0.785
        elif y>250.0 and y<300.0:
            self.orientation = -1.0
        elif y>=200.0 and y<=300.0:
            self.orientation = -1.22
        elif y>=160.0 and y<200.0:
            self.orientation = -1.5708
        elif y<160:
            exit()

        # Calculate the different commands    
        P_y = y - self.length3 * math.cos(self.orientation)
        P_z = z - self.length3 * math.sin(self.orientation)
        
        self.joint3_angle = - math.acos(((P_z**2 + P_y**2) - (self.length1**2 + self.length2**2))/(2*self.length1*self.length2))
        
        self.joint2_angle = math.atan2(P_z , P_y) -  math.atan2(self.length2 * math.sin(self.joint3_angle),(self.length1 + self.length2 * math.cos(self.joint3_angle)))

        self.joint4_angle = self.orientation - (self.joint2_angle + self.joint3_angle)

        self.joint2_angle -= math.pi/2

        rospy.sleep(0.5)

        self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
        self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
        self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor 

        print("command:",self.joint2_angle,"state:",self.joint2_state)
        print("command:",self.joint3_angle,"state:",self.joint3_state)
        print("command:",self.joint4_angle,"state:",self.joint4_state)
    
    
    
    def publish_commands(self):
        
                       
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)
                       
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)


        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)

        rospy.sleep(self.command2_duration/1000)
          
        rospy.sleep(1)
        gripper_command = Float64()
        gripper_command.data = -0.2
        self.gripper_pub.publish(gripper_command)
        rospy.sleep(1)
    
    def return_to_origin (self):
        self.joint1_angle = -1.570
        self.joint2_angle = 0.0
        self.joint3_angle = -1.35
        self.joint4_angle = -0.78

        self.command1_duration = abs(self.joint1_angle - 0.0) * self.time_factor
        self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
        self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
        self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor
        
        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)
        rospy.sleep(self.command2_duration/1000)
        
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)
        
        
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)
        
        command1 = CommandDuration()
        command1.data = self.joint1_angle
        command1.duration = self.command1_duration
        self.joint1_command_pub.publish(command1)
        rospy.sleep(self.command1_duration/1000)

        gripper_command = Float64()
        gripper_command.data = -1.3
        self.gripper_pub.publish(gripper_command)
        rospy.sleep(1)

        self.joint1_angle = 0.0
        self.joint2_angle = 0.5
        self.joint3_angle = -1.35
        self.joint4_angle = -1.76

        self.command1_duration = abs(self.joint1_angle - -1.570) * self.time_factor
        self.command2_duration = abs(self.joint2_angle - self.joint2_state) * self.time_factor
        self.command3_duration = abs(self.joint3_angle - self.joint3_state) * self.time_factor 
        self.command4_duration = abs(self.joint4_angle - self.joint4_state) * self.time_factor
        
        command1 = CommandDuration()
        command1.data = self.joint1_angle
        command1.duration = self.command1_duration
        self.joint1_command_pub.publish(command1)
        rospy.sleep(self.command1_duration/1000)

        command2 = CommandDuration()
        command2.data = self.joint2_angle
        command2.duration = self.command2_duration
        self.joint2_command_pub.publish(command2)
        rospy.sleep(self.command2_duration/1000)
        
        command3 = CommandDuration()
        command3.data = self.joint3_angle
        command3.duration = self.command3_duration
        self.joint3_command_pub.publish(command3)
        rospy.sleep(self.command3_duration/1000)
        
        
        command4 = CommandDuration()
        command4.data = self.joint4_angle
        command4.duration = self.command4_duration
        self.joint4_command_pub.publish(command4)
        rospy.sleep(self.command4_duration/1000)
   




        
 
    


if __name__ == '__main__':

    y = 300.0
    z = -135.0

    rospy.init_node('ik_controller') 

    controller = RoboticArm()

    controller.calculate_commands(y, z)

    controller.publish_commands()

    controller.return_to_origin()
    




