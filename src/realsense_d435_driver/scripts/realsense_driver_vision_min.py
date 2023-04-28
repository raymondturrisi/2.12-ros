#!/usr/bin/env python

import rospy
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField

def numpy_to_image_msg(numpy_img, encoding, frame_id):
    img_msg = Image()
    img_msg.header.stamp = rospy.Time.now()
    img_msg.header.frame_id = frame_id
    img_msg.height = numpy_img.shape[0]
    img_msg.width = numpy_img.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.step = numpy_img.strides[0]
    img_msg.data = numpy_img.tostring()
    return img_msg


def main():
    rospy.init_node('realsense_d435', anonymous=True)
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure depth and color streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    # Start streaming
    pipeline.start(config)

    # Create publishers for depth and color images
    color_pub = rospy.Publisher('/realsense/color/image_raw', Image, queue_size=100)

    rate = rospy.Rate(15)  # 30 Hz

    try:
        while not rospy.is_shutdown():
            # Get frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            # Convert frames to images
            color_image = np.asanyarray(color_frame.get_data())

            # Publish depth and color images
            color_pub.publish(numpy_to_image_msg(color_image, "bgr8", "realsense_color_frame"))
       
            rate.sleep()

    finally:
        pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
