#!/usr/bin/env python

from __future__ import print_function
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
import navigation

class Follow_Path():
    ''' Pinbot path following based on A* path planning output'''

    def __init__(self):
        self.pixels = navigation.get_path()
        self.coords = []
        self.pixel_to_coords()
        self.current_ind = 5
        self.directed_angle = None
        self.done = False
        self.last_distance = 10
    
    def pixel_to_coords(self):
        pixel_start = self.pixels[0]
        for i in range(0, len(self.pixels)):
            self.coords.append(((self.pixels[i][0]-pixel_start[0]) * 0.05, -(self.pixels[i][1]-pixel_start[1]) * 0.05))        

	def get_directed_angle(self):
		mini_goal = self.coords[self.current_ind]
		distance = math.sqrt((self.pose[0] - mini_goal[1])**2 + (-self.pose[1] - mini_goal[0])**2)
		if distance < 0.25 or round(self.last_distance, 2) < round(distance, 2):
			if self.current_ind != -1 and self.current_ind != len(self.coords) - 1:
				self.last_mini_goal = mini_goal
				self.current_ind += 5
				self.last_distance = 10
			else:
				self.done = True
			try:
				mini_goal = self.coords[self.current_ind]
			except IndexError:
				self.current_ind = -1 
				mini_goal = self.coords[-1]
			xdif = mini_goal[0] - self.last_mini_goal[0]
			ydif = mini_goal[1] - self.last_mini_goal[1]
			self.vector_goal = [xdif / math.hypot(xdif, ydif), ydif / math.hypot(xdif, ydif)]
		else:
			self.last_distance = distance
		vector_orientation = [-math.sin(self.orientation), math.cos(self.orientation)]
		self.directed_angle = math.degrees(math.atan2(self.vector_goal[1], self.vector_goal[0]) - math.atan2(vector_orientation[1], vector_orientation[0]))
		if self.directed_angle > 180:
			self.directed_angle = -(360 - self.directed_angle)
		print (distance, vector_orientation, self.vector_goal, self.directed_angle, self.current_ind)
	
    def odom_received(self, odom_data):
        orient = odom_data.pose.pose.orientation
        pose = odom_data.pose.pose.position
        self.pose = (pose.x, pose.y)
        rotation = (orient.x, orient.y, orient.z, orient.w)
        self.orientation = tf.transformations.euler_from_quaternion(rotation)[2]
        self.get_directed_angle()
    
    def main(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('/odom', Odometry, self.odom_received)
        rospy.init_node('follow_path', anonymous=True)
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.done == False: 
            if self.directed_angle != None:
                if math.fabs(self.directed_angle) > 8:
                    msg = Twist(angular=Vector3(z = self.directed_angle * 0.01))
                else:
					msg = Twist(linear=Vector3(x=0.1), angular=Vector3(z= self.directed_angle * 0.05))
                pub.publish(msg)
		if self.done == True:
			print("Done")
			msg = Twist(linear=Vector3(x=0), angular=Vector3(z=0))
			pub.publish(msg)
			r.sleep()

if __name__ == '__main__':
		myProgram = Follow_Path()
		myProgram.main()