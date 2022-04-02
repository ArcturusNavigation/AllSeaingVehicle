#!/usr/bin/env python

import os
import sys

import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
CAMERA_TOPIC = 'zed/rgb/image_rect_color'
CAMERA_FRAME = 'map'


class FakeCamera:
    def __init__(self, src, frame_rate=1, loop=True):
        self.src = self.parse(src)
        self.frame_rate = frame_rate
        self.loop = loop
        self.frame_count = 0
        self.frame_time = 1.0 / frame_rate

    def parse(self, src):
        if os.path.isfile(src):
            raise NotImplementedError(
                "Video files are not yet supported")  # TODO
        elif os.path.isdir(src):
            return [cv.imread(os.path.join(src, f)) for f in os.listdir(src) if f.endswith((".jpg", ".png"))]
        raise ValueError("Invalid source: {}".format(src))

    def read(self):
        if self.loop:
            self.frame_count = (self.frame_count + 1) % len(self.src)
        else:
            self.frame_count = min(self.frame_count + 1, len(self.src) - 1)
        return self.src[self.frame_count]


def main():
    rospy.init_node('fake_camera')
    rate = 1
    if len(sys.argv) == 2:
        rate = int(sys.argv[1])
    r = rospy.Rate(rate)
    camera = FakeCamera('./src/sensor_suite/images/redoverlays')
    pub = rospy.Publisher(CAMERA_TOPIC, Image, queue_size=1)
    while not rospy.is_shutdown():
        frame = camera.read()
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = CAMERA_FRAME
        msg.height, msg.width, _ = frame.shape
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tostring()
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
