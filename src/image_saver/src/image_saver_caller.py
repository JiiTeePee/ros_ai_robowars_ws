#! /usr/bin/python3

import rospy
from std_srvs.srv import Empty

rospy.init_node("image_saver_caller")                                # käynnistetään node, pitää antaa nimi
rate = rospy.Rate(0.5)                                               # kuinka usein tallennetaan, 0.5 / s
while not rospy.is_shutdown():
    rospy.wait_for_service('/camera_controller/save')               # service server client liityntä varmistetaan
    saver = rospy.ServiceProxy('/camera_controller/save', Empty)    # topic, tyyppi Empty
    saver()                                                         # soitetaan
    rate.sleep()

# topicit monelta monella
# placeholder value mahdollinen