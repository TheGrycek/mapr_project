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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as Path_map
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
old_image = np.zeros(3600)
loaded = False

cwd = str(Path(__file__).resolve().parent.parent)
image_dir2 = cwd + '/images/images2'
image_dir1 = cwd + '/images/images1'
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
      #start_x = 5
      #end_x = 35
      #start_y = 5
      #end_y = 55

   #start_z = data_tmp[offset + start_y + stride1 * start_x + 0]
   #end_z = data_tmp[offset + end_y + stride1 * end_x + 0]

   map_cpy.info = elev_map.info
   map_cpy.layers = elev_map.layers
   map_cpy.basic_layers = elev_map.basic_layers
   map_cpy.outer_start_index = elev_map.outer_start_index
   map_cpy.inner_start_index = elev_map.inner_start_index

def callback_path(path):
   global img_number,image_dir1, image_dir2, stride0, stride1, cols, rows, offset, data_tmp, old_image, loaded

   new_image = np.array(list(data_tmp))
   max = np.amax(data_tmp)
   min = np.amin(data_tmp)
   image_name = 'map%u.png' % img_number

   for pix in range(new_image.size):
      # map pixels value range 5 - 255
       new_image[pix] = ((new_image[pix] - min) / (max - min)) * (255 - 30) + 30
       new_image[pix] = np.uint8(round(new_image[pix]))

   #print("Start point:")
   #print(start_x, start_y)
   #print("End point:")
   #print(end_x, end_y)
   #print("Path points:")

   path_points = []  # tablica potrzebna do interpolacji sciezki
   old_image2 = new_image.copy()

   for pos in path.poses:
       posX = int(pos.pose.position.x / (-0.1))
       posY = int(pos.pose.position.y / (-0.1))

       #print(posX, posY)
       path_points.append((posX,posY))	
       #print(pos.pose.position.x, pos.pose.position.y)
       old_image[offset + posX + stride1 * posY + 0] = 0
       if posY | posX != 0:
           loaded = True

   #zdjecia z punktem startowym i koncowym
   koniec = len(path_points)

   if koniec:
	koniec = koniec-1

   old_image2[offset + path_points[0][0] + stride1 * path_points[0][1]] = 0	
   old_image2[offset + path_points[koniec][0] + stride1 * path_points[koniec][1]] = 0	
	
   img_write1 = old_image2.reshape(rows, cols)
   img_write2 = old_image.reshape(rows, cols)
   

   # interpolacja sciezki	
   start = path_points[0]	
   for pos in path_points:
   	cv.line(img_write2 ,start,pos,0,1)
	start = pos

   if loaded:
      if img_number != 0:
	 # zapis zdjecia z punktem startowym i koncowym
	  os.chdir(image_dir1)
	  cv.imwrite(image_name, img_write1)
	# zapis sciezki
	  os.chdir(image_dir2)
          cv.imwrite(image_name, img_write2)

      old_image = new_image
      img_number += 1	
  	
  

def mapListener():
   global start_x, start_y, end_x, end_y, start_z, end_z

   rospy.init_node('start_end', anonymous=True)
   rospy.Subscriber("/map_copy", msg.GridMap, callback_map)
   # spin() simply keeps python from exiting until this node is stopped
   pub_start_x = rospy.Publisher('/start_point_x', UInt8, queue_size=10)
   pub_end_x = rospy.Publisher('/end_point_x', UInt8, queue_size=10)
   pub_start_y = rospy.Publisher('/start_point_y', UInt8, queue_size=10)
   pub_end_y = rospy.Publisher('/end_point_y', UInt8, queue_size=10)
   rospy.Subscriber("/planned_path", Path_map, callback_path)

   rate = rospy.Rate(0.5)

   while not rospy.is_shutdown():
      # start_str = '%u, %u, %f' % (start_x, start_y, start_z)
      # end_str = '%u, %u, %f' % (end_x, end_y, end_z)
      # rospy.loginfo(start_str)
      # rospy.loginfo(end_str)

      pub_start_x.publish(start_x)
      pub_start_y.publish(start_y)
      pub_end_x.publish(end_x)
      pub_end_y.publish(end_y)

      rate.sleep()

if __name__ == '__main__':
   mapListener()
