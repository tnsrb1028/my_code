#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from turtlesim.msg import Pose

bridge = CvBridge()
img = np.empty(shape=[0])
x, y, theta = 0, 0, 0

control_msg = Twist()

def image_callback(img_data):
    global bridge
    global img
    img = bridge.imgmsg_to_cv2(img_data, "bgr8")


def pose_callback(pose_data):
    global x
    global y
    global theta

    x = pose_data.x
    y = pose_data.y
    theta = pose_data.theta


def control_msg_publish(linear_x, angular_z):
    	control_msg.linear.x = linear_x
    	control_msg.linear.y = 0
    	control_msg.linear.z = 0
    	control_msg.angular.x = 0
    	control_msg.angular.y = 0
    	control_msg.angular.z = angular_z

        pub.publish(control_msg)


if __name__ == "__main__":
    rospy.init_node("foscar_project")
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    time.sleep(1)

    while not rospy.is_shutdown():
        # 이미지 hsv 변환해서 저장하기
        lower_green = (45, 55, 55)
        upper_green = (85 , 255, 255)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, lower_green, upper_green)
        img_flip = cv2.flip(img_mask,1)
        #img_result = cv2.bitwise_and(img, img, mask=img_red)
        cv2.imshow("Screen",img_flip)

        non = img_flip.nonzero()

        if len(img_flip.nonzero()[0])*len(img_flip.nonzero()[1])/(img.shape[0]*img.shape[1]) > 0.1:
            result_x = int(sum(non[1]) / len(non[1]))
            result_y = int(sum(non[0]) / len(non[0]))
            target_x = result_x / 58
            target_y = 11-(result_y / 44)

            print(target_x)
            print(target_y)
            # 거북이 제어
            a = target_x-x
            b = target_y-y
            c = float(a*a+b*b)
            d = math.sqrt(c)
            v = 0.7*d
            z = 3*(math.atan2(target_y-y , target_x-x)-theta)
            control_msg_publish(v,z)
            # 거북이 pose 정보 출력

            print("x : ", x)
            print("y : ", y)
            #print("theta : ", theta)
            #print("")

        else:
            print('Not detected')

        # 격자무늬 시각화
        cv2.line(img, (0, 160), (639, 160), (0, 0, 0), 1)
        cv2.line(img, (0, 320), (639, 320), (0, 0, 0), 1)
        cv2.line(img, (210, 0), (210, 480), (0, 0, 0), 1)
        cv2.line(img, (420, 0), (420, 480), (0, 0, 0), 1)

        # 비디오 화면 띄우기
        # 동시에 여러 창 띄우는것도 가능
        #cv2.imshow("image", img)
        #cv2.imshow("hsv_image", img_hsv)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
