#!/usr/bin/env python
from __future__ import print_function
import sys
import os
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from depth_estimation import estimate_depth

if __name__ == '__main__':
    rospy.init_node('kitti_publisher')
    cv_bridge = CvBridge()
    left_pub = rospy.Publisher("left_image", Image, queue_size=100)
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=100)
    right_pub = rospy.Publisher("right_image", Image, queue_size=100)
    depth_pub = rospy.Publisher("depth_image", Image, queue_size=100)
    depth_visual_pub = rospy.Publisher("depth_visual_image", Image, queue_size=100)

    path_name = "/home/hamdaan/Downloads/2011_09_26_drive_0036_sync/2011_09_26/2011_09_26_drive_0036_sync"
    image_index = 0
    rate = rospy.Rate(5)
    #cv2.namedWindow("left")
    #cv2.moveWindow("left", 100, 100)
    cv2.waitKey(0)
    left_img_list = os.listdir(path_name+"/image_00/data")
    left_img_list.sort()
    right_img_list = os.listdir(path_name+"/image_01/data")
    right_img_list.sort()
    while (not rospy.is_shutdown()):
        print(image_index)
        left_img_path = path_name + "/image_00/data/" + left_img_list[image_index]
        right_img_path = path_name + "/image_01/data/" + right_img_list[image_index]
        depth_path = path_name + "/depth_0/%06d.npy" % image_index
        #if (not os.path.isfile(left_img_path)) or (not os.path.isfile(right_img_path)):
        #    print(left_img_path)
        #    break

        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)
        depth_img = estimate_depth(left_img, right_img)
        #depth_np = np.load(depth_path)
        #depth_np = 386.1448 / depth_np  #00-02
        # depth_np = 379.8145 / depth_np  #04-12
        #depth_visual = np.clip(depth_np, 0.5, 60)
        #depth_visual = depth_visual / 60.0 * 255.0
        #depth_visual = depth_visual.astype(np.uint8)
        #col = cv2.applyColorMap(depth_visual, cv2.COLORMAP_RAINBOW)

        pub_ros_time = rospy.get_rostime()
        left_msg = cv_bridge.cv2_to_imgmsg(left_img, "rgb8")
        left_msg.header.stamp = pub_ros_time
        left_pub.publish(left_msg)
        pub.publish(left_msg)
        

        right_msg = cv_bridge.cv2_to_imgmsg(right_img, "rgb8")
        right_msg.header.stamp = pub_ros_time
        right_pub.publish(right_msg)

        depth_msg = cv_bridge.cv2_to_imgmsg(depth_img, "8UC1")
        depth_msg.header.stamp = pub_ros_time
        depth_pub.publish(depth_msg)

        #depth_visual_msg = cv_bridge.cv2_to_imgmsg(col, "bgr8")
        #depth_visual_msg.header.stamp = pub_ros_time
        #depth_visual_pub.publish(depth_visual_msg)

        rate.sleep()
        image_index+=1

        cv2.imshow("left", left_img)
        input_key = cv2.waitKey(10)

        if input_key == 27:
            break
