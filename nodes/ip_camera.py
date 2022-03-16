#!/usr/bin/env python
import sys
import cv2
import argparse
import roslib
import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 


class IPCamera(object):
    def __init__(self, url):
        try:
            self.vcap = cv2.VideoCapture(url)
        except:
            rospy.logerr('Unable to open ip camera stream: ' + str(url))
            sys.exit()
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()


def crop(frame_to_crop):
    cropped_frame = frame_to_crop[0:720, 150:1150]
    return cropped_frame


if __name__ == '__main__':

    url = 'http://admin:admin@192.168.0.94:8008'
    rospy.init_node('ip_camera', anonymous=True)
    
    print 'Opening ip camera'
    ip_camera = IPCamera(url)
    print 'Successfully opened ip camera'
    
    while not rospy.is_shutdown() and ip_camera.vcap.isOpened():
        ret, frame = ip_camera.vcap.read()
        if not ret:
            print 'Could not read frame'
            break
        if not ip_camera.vcap.isOpened():
            print 'Camera not opened'

        img_msg = ip_camera.bridge.cv2_to_imgmsg(crop(frame), "bgr8")
        img_msg.header.stamp = rospy.get_rostime()
        ip_camera.image_pub.publish(img_msg)

    ip_camera.vcap.release()
    cv2.destroyAllWindows()
