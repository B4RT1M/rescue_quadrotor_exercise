#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import tf
import numpy as np
import threading

class controller():
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.update_pose)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist,queue_size = 10 )
        # define a publisher that publishes Twist messages on the /cmd_vel topic
        self.pose_const = 1
        self.ang_const = 0.5

    def update_pose(self, data):
        #update current pose of the robot
        self.pose_stamped = data
        self.roll, self.pitch, self.yaw=self.quaternion_to_euler(data.pose)

    def get_goal(self):
        self.goal_x = input("Set your x goal:")
        self.goal_y = input("Set your y goal:")
        self.goal_z = input("Set your z goal:")
        angle_deg = input("Set your angle goal:")
        self.goal_angle = angle_deg*np.pi/180
        print("goal selected")

    def get_goal_cont(self):
        while not rospy.is_shutdown():
            self.get_goal()

    def quaternion_to_euler(self, robot_pose):
        #transforms robot position from quaternion to roll, pitch, yaw
        quaternion = (
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def control_law(self):
        #p_controller
        #define a control law here, use the "self.goal" varibles and "self.pose_stamped" variables here
		
		# calculate the error for x and y 
        dx = self.goal_x -self.pose_stamped.pose.position.x
        dy = self.goal_y -self.pose_stamped.pose.position.y
		
		# calculate the error for the z axes and the angle 
        dz = self.goal_z -self.pose_stamped.pose.position.z
        dw = self.goal_angle -self.yaw
		
		# now we transform the error for x and y 
		ux = (np.cos(self.yaw)*dx) + (np.sin(self.yaw)*dy)
		uy = (-(np.sin(self.yaw)*dx)) + (np.cos(self.yaw)*dy)
		
		# apply the constant p factor: 
		ux = self.pose_const*ux
		uy = self.pose_const*uy
		uz = self.pose_const*uz
		uw = self.ang_const*uw


        #return the control values
        return ux,uy,uz,uw
	print ux, uy

    def move2goal(self):
        while not rospy.is_shutdown():
            ux,uy,uz,uw = self.control_law()
			#define a Twist message here, apply the values from the control lab and publish it to cmd_vel
            twistMsg = Twist()
            twistMsg.linear.x = ux
            twistMsg.linear.y = uy
            twistMsg.linear.z = uz 
            twistMsg.angular.z = uw
            self.vel_publisher.publish(twistMsg)
        rospy.spin()
            



if __name__ == '__main__':
    try:
        rospy.init_node('p_controller')
        p_controller = controller()
        rospy.sleep(1) # waiting for the initial pose update
        p_controller.get_goal()
        t1 = threading.Thread(name='thread1', target=p_controller.get_goal_cont)
        t2 = threading.Thread(name='thread2', target=p_controller.move2goal)
        t1.start()
        t2.start()
    except rospy.ROSInterruptException: pass
