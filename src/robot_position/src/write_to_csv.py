#! /usr/bin/python3


import rospy, random, math, message_filters             # 
from nav_msgs.msg import Odometry                       # asentotieto

from sensor_msgs.msg import Range

from std_srvs.srv import Empty
import angles
import csv
#from rospy import service
from tf.transformations import euler_from_quaternion    # otetaan YAW käyttöön  

from sensor_msgs.msg import Imu                         # inertiamittuas .lis

#from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped

##################

class Robot_position:                                   # kirjoitetaan luokkana

    def __init__(self):
        self.node = rospy.init_node("position_from_ultrasonic") # uusi node

        self.reset_simulation_call = rospy.ServiceProxy('/gazebo/reset_simulaton',Empty) # empty:ei lähetetä mitään
        # jos jää nurkkaan jumittamaan, resetoidaaan kummankin paikat

        self.us1_sub = message_filters.Subscriber('/ultrasonic1',  Range)   # subscibe , tieto sisään, mitä topiccia kuunnellaan, tyyppi Range
        self.us2_sub = message_filters.Subscriber('/ultrasonic2',  Range)
        self.us3_sub = message_filters.Subscriber('/ultrasonic3',  Range)
        self.us4_sub = message_filters.Subscriber('/ultrasonic4',  Range)
        self.us5_sub = message_filters.Subscriber('/ultrasonic5',  Range)
        self.us6_sub = message_filters.Subscriber('/ultrasonic6',  Range)
        self.odom_sub = message_filters.Subscriber('/robot1/odom', Odometry) 
        # # asento ja ground truth tieto paikasta, topic odom, tyyppi odometry   
        #self._sub = message_filters.Subscriber('/robot1/imu', Imu)    #
       
        self.subs = message_filters.ApproximateTimeSynchronizer([self.us1_sub,self.us2_sub,self.us3_sub,self.us4_sub,self.us5_sub,self.us6_sub, self.odom_sub], queue_size=1, slop=0.9, allow_headerless=True)  

        #self.subs = message_filters.ApproximateTimeSynchronizer([           # kaikkiin arvo ja yksi callback, kaikki sensoriarvot kerralla
               # self.us1_sub, self.us2_sub, self.us3_sub,                         # kaappaa kaikki viestit
                #self.us4_sub, self.us5_sub, self.us6_sub,                           # lista mitä haluttiin
                #self.odom_sub], 
                #queue_size=1, slop=0.9,allow_headerless=True)                   # kuinka monto viestityyppiä jonossa
                                                                            # slop : kuinka iso viestien ero voi olla sekunteina
                                                                            # allow: voidaanko ottaa ilman headeria
        self.subs.registerCallback(self.sensor_cb)

        self.positions_file = open('robot1_positions.csv', mode ='w')          # avataan tiedosto mihin kirjoitetaan, ylikirjoitta aian
        self.positions_writer = csv.writer(self.positions_file, delimiter=",") # erotellaan arvot pilkulla

        self.reset_counter = 0                     
        self.write_to_csv_counter = 0

    # callback
    def sensor_cb(self, us1_sub, us2_sub, us3_sub, us4_sub, us5_sub, us6_sub, odom_sub):

        self.reset_counter += 1
        self.write_to_csv_counter += 1
        orientation_in_quarternions = (
            odom_sub.pose.pose.orientation.x,
            odom_sub.pose.pose.orientation.y,
            odom_sub.pose.pose.orientation.z,
            odom_sub.pose.pose.orientation.w)

        orientation_in_euler = euler_from_quaternion(orientation_in_quarternions) # quater.. to euler

        roll  = orientation_in_euler[0]
        pitch = orientation_in_euler[1]
        yaw   = orientation_in_euler[2]

        yaw_radians = angles.normalize_angle_positive(yaw)
        ground_truth_x = odom_sub.pose.pose.position.x
        ground_truth_y = odom_sub.pose.pose.position.y

        if self.write_to_csv_counter > 19:                      # joka 20:a

            self.positions_writer.writerow([yaw_radians, us1_sub.range, us2_sub.range, us3_sub.range, us4_sub.range, us5_sub.range, us6_sub.range, ground_truth_x, ground_truth_y])
            #self.positions_writer.writerow([
            #yaw_radians, us1_sub.range, us2_sub.range, 
            #us3_sub.range,us4_sub.range, us5_sub.range,         # otetaann  mittaukset ja oikeat arvot x ja y
            #us6_sub.range, ground_truth_x, ground_truth_y])

            self.write_to_csv_counter = 0

                        
        if self.reset_counter > 10000:

            self.reset_simulation_call()                        # käynnnistää uudelleen
            self.reset_counter = 0

if __name__ == '__main__':
    print("start")
    Robot_position()
    rospy.spin()