#!/usr/bin/env python

import rospy
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

'''
from tkinter import *
import threading
import math


global tk
tk = Tk()
global l_h, u_h, l_s, u_s, l_v, u_v
l_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, lower', orient = HORIZONTAL)
l_h.pack()
u_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, upper', orient = HORIZONTAL)
u_h.pack()
u_h.set(255)
l_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, lower', orient = HORIZONTAL)
l_s.pack()
u_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, upper', orient = HORIZONTAL)
u_s.pack()
u_s.set(255)
l_v = Scale(tk, from_ = 0, to = 255, label = 'Value, lower', orient = HORIZONTAL)
l_v.pack()
u_v = Scale(tk, from_ = 0, to = 255, label = 'Value, upper', orient = HORIZONTAL)
u_v.pack()
u_v.set(255)
'''

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
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create publishers for depth and color images
    depth_pub = rospy.Publisher('/realsense_ur5/depth/image_raw', Image, queue_size=100)
    color_pub = rospy.Publisher('/realsense_ur5/color/image_raw', Image, queue_size=100)
    filtered_pub = rospy.Publisher('/realsense_ur5/color/image_filt', Image, queue_size = 100)
    
    # Create publisher for distance and position information
    tri_loc_pub = rospy.Publisher('/realsense_ur5/depth/tri_loc', Float32MultiArray, queue_size = 10)
    rate = rospy.Rate(15)  # 30 Hz

    try:
        while not rospy.is_shutdown():
            # Get frames   
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            #tk.update()
            
            # Get distance of a pixel
            x = 320
            y = 240
            dpt_frame = pipeline.wait_for_frames().get_depth_frame().as_depth_frame()
            pixel_distance_in_meters = dpt_frame.get_distance(x, y)
            #print('Distance of (400, 300): ', pixel_distance_in_meters)
            #rospy.loginfo(pixel_distance_in_meters)
            #distance_pub.publish(pixel_distance_in_meters)
            
            # Convert frames to images
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            num_rows = depth_image.shape[0] # 480
            num_cols = depth_image.shape[1] # 640

            # Publish depth and color images
            depth_pub.publish(numpy_to_image_msg(depth_image, "16UC1", "realsense_depth_frame"))
            color_pub.publish(numpy_to_image_msg(color_image, "bgr8", "realsense_color_frame"))
            
            
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            
            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
            	resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            	images = resized_color_image
            else:
            	images = color_image
            
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)
            
            
            # Convert to HSV
            hsv_image = cv2.cvtColor(images, cv2.COLOR_BGR2HSV)
            
            # HSV Thresholding
            lower_bound_HSV = np.array([0, 26, 143])
            upper_bound_HSV = np.array([98, 255, 255])
            #lower_bound_HSV = np.array([l_h.get(), l_s.get(), l_v.get()])
            #upper_bound_HSV = np.array([u_h.get(), u_s.get(), u_v.get()])
            
            #Threshold
            mask_HSV = cv2.inRange(images, lower_bound_HSV, upper_bound_HSV)
            
            # Display image
            disp_image_HSV = cv2.bitwise_and(images, images, mask = mask_HSV)
            cv2.imshow("HSVThresholding", disp_image_HSV)
            cv2.waitKey(3)
            
            h, s, gray = cv2.split(disp_image_HSV) # v1 is the gray image
            ret,thresh = cv2.threshold(gray,70,255,0)
            # Display the Binary Image
            cv2.imshow("Binary Image", thresh)
            cv2.waitKey(3)
            

            
            loc = []
            for j in range(num_cols):
            	for i in range(num_rows):
            		if thresh[i][j] > 0.5:
            			loc.append((i,j))
            x_avg=0
            y_avg=0
            for pxl in loc:
            	x_avg += pxl[0]/len(loc)
            	y_avg += pxl[1]/len(loc)
            	 
            
            #for (x, y) in loc:
            triloc=Float32MultiArray()
            triloc.data=[x_avg,y_avg,pixel_distance_in_meters]
            tri_loc_pub.publish(triloc)
            print(triloc)


            filtered_pub.publish(numpy_to_image_msg(disp_image_HSV, "bgr8", "realsense_filtered_frame"))
            
            
            rate.sleep()

    finally:
        pipeline.stop()
        
        
class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
