#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64        # moottorille
from geometry_msgs.msg import Twist     # turtle, tele_op

class Robot_manual_drive:
    def __init__(self):                 # luokka objekti luodaan
            
        self.node = rospy.init_node("robot1_manual_drive")

        self.velocity_l_publisher = rospy.Publisher('/robot1/wheel_l_velocity_controller/command', Float64, queue_size=1)
        self.velocity_r_publisher = rospy.Publisher('/robot1/wheel_r_velocity_controller/command', Float64, queue_size=1)

        self.command_sub = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.callback)

        self.vel_l_msg = Float64()
        self.vel_r_msg = Float64()
        self.last_calback_timer = rospy.Time.now()
        self.send_zero_velocity_timer = rospy.Timer(rospy.Duration(0.1), self.send_zero_velocity)

    def callback (self, data):
        self.send_velocity(data.linear.x * 5 + data.angular.z * 2 , data.linear.x * 5 + data.angular.z * -2)
        self.last_calback_timer = rospy.Time.now()

    def send_zero_velocity(self, event=None):
        if rospy.Time.now().secs > self.last_calback_timer.secs + 0.1:
            self.send_velocity(0,0) # 0,1 s välein pitää tulla komentoja

    def send_velocity(self,vel_l,vel_r):
        self.vel_l_msg.data = vel_l
        self.vel_r_msg.data = vel_r

        self.velocity_l_publisher.publish(self.vel_l_msg)
        self.velocity_r_publisher.publish(self.vel_r_msg)

if __name__ == '__main__':
    Robot_manual_drive()
    while not rospy.is_shutdown():
        rospy.spin()