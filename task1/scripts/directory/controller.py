#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
 
import math
pi=math.pi

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray

hola_x = 0
hola_y= 0
hola_theta = 0

x_goals = [1, -1, -1, 1, 0]
y_goals = [1, 1, -1, -1, 0]
theta_goals = [pi/4, 3*pi/4, -3*pi/4, -pi/4, 0]

def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)



def odometryCb(msg):
	global hola_theta, hola_x, hola_y
	# Write your code to take the msg and update the three variables
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	o = msg.pose.pose.orientation
	(x,y,z) = euler_from_quaternion([o.x,o.y,o.z,o.w])
	hola_theta = z


def main():
	
	rospy.init_node('controller', anonymous=True)

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('odom', Odometry,odometryCb, queue_size=10)
	
	vel = Twist()
	
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	
	rate = rospy.Rate(30)

    
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    

	kp1 = 7
	kp2 = 3
	
	while not rospy.is_shutdown():
		for i in range(len(x_goals)):
			x_d = x_goals[i]
			y_d = y_goals[i]
			theta_d = theta_goals[i]
			print(x_d,y_d,theta_d)
			
		
			while 1:
				
	
				msg = Odometry()
				
				ex = x_d - hola_x
				ey = y_d - hola_y
				etheta = theta_d - hola_theta
				
				
				vel_x = 0
				vel_y = 0
				vel_z = 0
				vel_x_gframe = -kp2*ex
				vel_y_gframe = -kp2*ey
				if (abs(etheta) > 5*pi/180):
					vel_z = kp1*etheta
				else:
					vel_z = 2.5*kp1*etheta
				if (vel_z > 10):
					vel_z = 10
				if (vel_z < -10):
					vel_z = -10
				print(hola_theta*180/pi)
				vel_x_base = -math.cos(-hola_theta) * vel_x_gframe + math.sin(-hola_theta) * vel_y_gframe
				vel_y_base = -math.sin(-hola_theta) * vel_x_gframe - math.cos(-hola_theta) * vel_y_gframe
		
				vel.linear.x = vel_x_base
				vel.linear.y = vel_y_base
				vel.angular.z = vel_z
				msg = Odometry()
		
				pub.publish(vel)
				
				rate.sleep()
				if hola_x> x_d-0.01 and hola_x<x_d+0.01 and hola_y > y_d-0.01 and hola_y<y_d+0.01 and etheta> -0.01 and etheta< 0.01:
					print(hola_x,hola_y,hola_theta)
					vel.linear.x =0
					vel.linear.y=0
					vel.angular.z = 0
					pub.publish(vel)
					rospy.sleep(2)
					print("done")
					break

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass