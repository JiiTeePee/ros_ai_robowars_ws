#! /usr/bin/python3

from genpy import message
import rospy , message_filters
from nav_msgs.msg import Odometry
from rospy.topics import Message
from sensor_msgs.msg import Range
import angles
# odo imu tulostaa quartenioina
from tf.transformations import euler_from_quaternion, quaternion_about_axis
import numpy as np 
from tensorflow import keras

from geometry_msgs.msg import Pose


#ladataan opetettu model

#model = keras.models.load_model('~/ros_ai_robowars_ws/saved_model/robot1_model490epochs')
model = keras.models.load_model('/home/jiiteepee/ros_ai_robowars_ws/saved_model/robot1_model490epochs')

predict_data = np.zeros((1,7))      # array
print(predict_data.shape)
test_predictions = model.predict(predict_data)

 # tehdää tyhjä array, katsottiin dimensiot, jos kaikki nollia mitä plauttaa

class Robot_position:
    def __init__(self):
        self.node = rospy.init_node('robot1_position_node')

        self.us1_sub =message_filters.Subscriber('/ultrasonic1', Range)
        self.us2_sub =message_filters.Subscriber('/ultrasonic2', Range)
        self.us3_sub =message_filters.Subscriber('/ultrasonic3', Range)
        self.us4_sub =message_filters.Subscriber('/ultrasonic4', Range)
        self.us5_sub =message_filters.Subscriber('/ultrasonic5', Range)
        self.us6_sub =message_filters.Subscriber('/ultrasonic6', Range)
        self.odom_sub =message_filters.Subscriber('/robot1/odom', Odometry)

        # ottaa sisään kaikki viestit

        self.subs = message_filters.ApproximateTimeSynchronizer([
            self.us1_sub, self.us2_sub, self.us3_sub, self.us4_sub,
            self.us5_sub, self.us6_sub, self.odom_sub ],queue_size=1, slop=0.9, allow_headerless=True) #kuinka kauan odottaa muita viestehä

        self.predict_data = np.zeros((1,7))

        self.model = keras.models.load_model('/home/jiiteepee/ros_ai_robowars_ws/saved_model/robot1_model490epochs')
        self.predict_pose_publisher = rospy.Publisher('robot1/predicted_pose', Pose ,queue_size=1)

    def sensor_cb(
        self, us1_sub,us2_sub, us3_sub,us4_sub,
        us5_sub, us6_sub,odom_sub):

        orientation_in_quaternions = (
            odom_sub.pose.pose.orientation.x,
            odom_sub.pose.pose.orientation.y,
            odom_sub.pose.pose.orientation.z,
            odom_sub.pose.pose.orientation.w)

        orientation_in_euler = euler_from_quaternion(orientation_in_quaternions)
        yaw = orientation_in_euler [2]
        yaw_radians = angles.normalize_angle_positive(yaw)

        ground_truth_x = odom_sub_pose.pose.position.x
        ground_truth_y = odom_sub_pose.pose.position.y

        self.predict_data [0][0] = yaw_radians
        self.predict_data [0][1] = us1_sub.range
        self.predict_data [0][2] = us2_sub.range
        self.predict_data [0][3] = us3_sub.range
        self.predict_data [0][4] = us4_sub.range
        self.predict_data [0][5] = us5_sub.range
        self.predict_data [0][6] = us6_sub.range

        test_predictions = self.model.predict(self.predict_data)
        print("predictions" + test_predictions)
        print("ground truth" + ground_truth_x + ground_truth_y)

        msg = Pose() #geometry msg pose, kaksi komponenttia, posientation ja orientation
        msg.position.x = test_predictions [0][0]
        msg.position.y = test_predictions [0][1]
        msg.position.z= 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 0
        self.predicted_pose_publisher.publish(msg)


if __name__ == '__main__':
    print("start")
    Robot_position()
    rospy.spin()






    # /include samk. robowars arena, no pit lauchiin

    #node type="predict_position.py pkg="robot_position" name="robot1_predict_position" outpuyt ="screen"/>