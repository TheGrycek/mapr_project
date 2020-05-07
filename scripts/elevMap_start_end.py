#!/usr/bin/env python
import rospy
import random as rd
import numpy as np
import cv2 as cv
import os
from grid_map_msgs import msg
from std_msgs.msg import UInt8
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from pathlib import Path

map_cpy = msg.GridMap()
start_x = np.uint8(0)
start_y = np.uint8(0)
start_z = np.float64(0)
end_x = np.uint8(0)
end_y = np.uint8(0)
end_z = np.float64(0)
data_tmp = []
stride0, stride1, cols, rows, offset = 0, 0, 0, 0, 0

cwd = str(Path(__file__).resolve().parent.parent)
image_dir = cwd + '/images'
img_number = 0

def callback_map(elev_map):
   global image_dir, img_number, start_x, end_x, start_y, end_y, start_z, \
      end_z, stride0, stride1, cols, rows, offset, data_tmp

   stride0 = elev_map.data[0].layout.dim[0].stride
   stride1 = elev_map.data[0].layout.dim[1].stride
   cols = elev_map.data[0].layout.dim[0].size
   rows = elev_map.data[0].layout.dim[1].size
   offset = elev_map.data[0].layout.data_offset
   map_cpy.data = elev_map.data
   data_tmp = list(map_cpy.data[0].data)

   start_x, start_y, end_x, end_y = 0, 0, 0, 0
   while(abs(start_x - end_x) + abs(start_y - end_y) <= 10):
      start_x = rd.randrange(0, rows, 1)
      end_x = rd.randrange(0, rows, 1)
      start_y = rd.randrange(0, cols, 1)
      end_y = rd.randrange(0, cols, 1)

   start_z = data_tmp[offset + start_y + stride1 * start_x + 0]
   end_z = data_tmp[offset + end_y + stride1 * end_x + 0]

   map_cpy.info = elev_map.info
   map_cpy.layers = elev_map.layers
   map_cpy.basic_layers = elev_map.basic_layers
   map_cpy.outer_start_index = elev_map.outer_start_index
   map_cpy.inner_start_index = elev_map.inner_start_index

   ###################################################

   new_image = np.array(data_tmp)
   max = np.amax(data_tmp)
   min = np.amin(data_tmp)
   image_name = 'map%u.png' % img_number
   img_number = img_number + 1

   for pix in range(new_image.size):
      # map pixels value range 5 - 255
       new_image[pix] = ((new_image[pix] - min) / (max - min)) * (255 - 5) + 5
       new_image[pix] = np.uint8(round(new_image[pix]))

   new_image[offset + start_y + stride1 * start_x + 0] = 0
   new_image[offset + end_y + stride1 * end_x + 0] = 0
   new_image = new_image.reshape(rows, cols)
   os.chdir(image_dir)
   cv.imwrite(image_name, new_image)

def callback_path(path):
   pass
   global img_number, image_dir, stride0, stride1, cols, rows, offset, data_tmp

   #TODO Zamienic wiadomosc typu path na punkty na mapie wysokosciowej
   # - zamienic wartosc pikseli odpowiadajacych punktom na 0

   new_image = np.array(list(data_tmp))
   max = np.amax(data_tmp)
   min = np.amin(data_tmp)
   image_name = 'map%u.png' % img_number
   img_number = img_number + 1

   for pix in range(new_image.size):
      # map pixels value range 5 - 255
       new_image[pix] = ((new_image[pix] - min) / (max - min)) * (255 - 5) + 5
       new_image[pix] = np.uint8(round(new_image[pix]))

   new_image[offset + start_y + stride1 * start_x + 0] = 0
   new_image[offset + end_y + stride1 * end_x + 0] = 0
   new_image = new_image.reshape(rows, cols)
   os.chdir(image_dir)
   cv.imwrite(image_name, new_image)


# def createPoints(start_x, start_y, start_z, end_x, end_y, end_z):
#    header = Header(stamp=rospy.Time.now(), frame_id="odom")
#    resolution = map_cpy.info.resolution
#    x_off = map_cpy.info.pose.position.x
#    y_off = map_cpy.info.pose.position.y
#    z_off = map_cpy.info.pose.position.z
#    start_point = Point(start_x * resolution + x_off, start_y * resolution + y_off, start_z + z_off)
#    end_point = Point(end_x * resolution + x_off, end_y * resolution + y_off, end_z + z_off)
#    start_point_stamped = PointStamped(header=header, point=start_point)
#    end_point_stamped = PointStamped(header=header, point=end_point)
#
#    return(start_point_stamped, end_point_stamped)

def mapListener():
   global start_x, start_y, end_x, end_y, start_z, end_z

   rospy.init_node('start_end', anonymous=True)
   rospy.Subscriber("/map_copy", msg.GridMap, callback_map)
   # spin() simply keeps python from exiting until this node is stopped
   pub_start_x = rospy.Publisher('/start_point_x', UInt8, queue_size=10)
   pub_end_x = rospy.Publisher('/end_point_x', UInt8, queue_size=10)
   pub_start_y = rospy.Publisher('/start_point_y', UInt8, queue_size=10)
   pub_end_y = rospy.Publisher('/end_point_y', UInt8, queue_size=10)
   # rospy.Subscriber("/path", Path, callback_path)

   rate = rospy.Rate(1)

   while not rospy.is_shutdown():
      start_str = '%u, %u, %f' % (start_x, start_y, start_z)
      end_str = '%u, %u, %f' % (end_x, end_y, end_z)
      rospy.loginfo(start_str)
      rospy.loginfo(end_str)

      pub_start_x.publish(start_x)
      pub_start_y.publish(start_y)
      pub_end_x.publish(end_x)
      pub_end_y.publish(end_y)

      rate.sleep()

if __name__ == '__main__':
   mapListener()
