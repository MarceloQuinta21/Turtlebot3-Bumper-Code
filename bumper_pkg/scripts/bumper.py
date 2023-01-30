#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlebot3_msgs.msg import SensorState
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import time

class MarkerPoint(object):
    def __init__(self):
        self.marker_pub = self.marker()
        self.rate = rospy.Rate(1)
        self.init_marker(index=0)

    def marker(self):
        # Creates the marker publisher to place a red dot on the map when the bumper is touched
        marker_pub = rospy.Publisher('/marker_topic', Marker, queue_size=1)
        return marker_pub

        
    def init_marker(self,index=0):
        # Initializes the marker parameters 
        self.marker = Marker()
        self.marker.header.frame_id ="map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = int(time.time())
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD

        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1

        self.marker.lifetime = rospy.Duration(0)

    def update_position(self, position):
        # Gets the position of the robot (the coordinates)
        self.marker.pose.position = position
        self.marker_pub.publish(self.marker)


class TurtleBot_Bumper():
    def __init__(self):
        self.bumper_pub = self.sensor_pub()
        self.bumper_sub = self.bumper_sensor_sub()
        self.marker_sub = self.position_sub()
        self.velocity_sub = self.velocity_sub()
        self.turn = Twist()
        self.turtle_point = Point()
        self.new_vel = 0
        self.timer = 0
        self.bumper_data()

    def sensor_pub(self):
        # Bumper publisher, to send data in case the bumper was touched or not
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        return pub

    def position_sub(self):
        # Subscribe to the odometry msg to gather the position and velocity of the robot
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        return odom_sub

    def velocity_sub(self):
        # Receive velocity data
        vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.velocity)
        return vel_sub

    def bumper_sensor_sub(self):
        # Find out what is the state of the bumper sensor
        sub = rospy.Subscriber('sensor_state', SensorState, self.bumper_input)
        return sub

    def callback(self, msg):
        # Gete the position of the robot in the x, y, and z coordinates
        self.turtle_point.x = msg.pose.pose.position.x
        self.turtle_point.y = msg.pose.pose.position.y
        self.turtle_point.z = msg.pose.pose.position.z

    def velocity(self, vel):
        self.new_vel = vel

    def bumper_input(self, sensor):
        # Assign the state of the bumper to self.state
        self.state = sensor.bumper
        if self.state == 1:
            # If front bumper was touched mark where it was touched and move backwards at an angle
            self.marker_object = MarkerPoint()
            self.marker_object.update_position(position=self.turtle_point)
            self.turn.linear.x = -0.1
            self.turn.angular.z = 0.75
            self.timer = time.time()
        elif self.state == 2:
            # If back bumper was touched mark where it was touched and move forward at an angle
            self.marker_object = MarkerPoint()
            self.marker_object.update_position(position=self.turtle_point)
            self.turn.linear.x = 0.1
            self.turn.angular.z = 0.75
            self.timer = time.time()
        elif self.timer + 3 < time.time():
            # If no bumper is touched keep moving
            self.turn.linear.x = 0.1 #self.new_vel.linear.x
            self.turn.angular.z = 0 #self.new_vel.angular.z
            #print(self.new_vel.linear.x)


    def bumper_data(self):
        rate = rospy.Rate(10)
        self.turn.linear.x = 0
        while not rospy.is_shutdown():
            self.bumper_pub.publish(self.turn)
            rate.sleep()


def main():
    rospy.init_node('bumper_node')
    try:
        bumper = TurtleBot_Bumper()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
