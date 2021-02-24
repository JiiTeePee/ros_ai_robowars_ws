#! usr/bin/python3

import rospy, random, math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64



def callback(data):                                 #otetaan nimellä data sissään
    vel_l_msg = Float64()                           # lähetetään kaksi eri viestiä, moottorille liikkumiskomento
    vel_r_msg = Float64()
    position_x = data.pose.pose.position.x          # kun callback tulee, message odometria otettu sisään nimellä data
    position_y = data.pose.pose.position.y          # dokumnetaatio muoto
    velocity_l = 0
    velocity_r = 0

    if position_x < 0.4:                            # alkuun yksinkertainen liikutus
        velocity_l = -10
        velocity_r = -3

    elif position_y < 0.4:
        velocity_l = -10
        velocity_r = -3

    elif position_x > 2.6:
        velocity_l = -3
        velocity_r = -10

    elif position_y > 2.6:
        velocity_l = -3
        velocity_r = -10  

    else:
        velocity_l = random.uniform(-0.1, -10)
        velocity_r = random.uniform(-0.1, -10)

    vel_l_msg.data = velocity_l
    vel_r_msg.data = velocity_r
    velocity_l_publisher.publish(vel_r_msg)                          # mitä julkaistaan
    velocity_r_publisher.publish(vel_r_msg)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node("random_walk_robot1")       # nimi, uusi node
        # tarvitaan kaksi publisehria, topic, message tyyppi:F64, uusin tyyppi käytössä
        velocity_l_publisher = rospy.Publisher('/robot1/wheel_l_velocity_controller/command', Float64, queue_size=1)
        velocity_r_publisher = rospy.Publisher('/robot1/wheel_r_velocity_controller/command', Float64, queue_size=1)
        rospy.Subscriber("/robot1/odom", Odometry, callback)    #kuunnellaan täältä paikka ja tehdään jotain kun tulee viesti

        rospy.spin()