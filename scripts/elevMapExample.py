# elevMapExample.py
#
#  Created on: June 15, 2020
#      Author: Kamil Miedzinski Mateusz Grycmacher
#	 Institute: Instute of Home, Poznan University of Technology

#!/usr/bin/env python
import rospy
from grid_map_msgs import msg # import occupancy grid data

map_cpy = msg.GridMap()

def callback(elev_map):

   # access the first layer (index 0) - elevation
   stride0 = elev_map.data[0].layout.dim[0].stride
   stride1 = elev_map.data[0].layout.dim[1].stride
   # dimension
   cols = elev_map.data[0].layout.dim[0].size
   rows = elev_map.data[0].layout.dim[1].size
   #offset
   offset = elev_map.data[0].layout.data_offset
   map_cpy.data = elev_map.data
  # crate list to edit tuple
   data_tmp = list(map_cpy.data[0].data)
   for i in range(cols):
       for j in range(rows):
           data_tmp[offset + i + stride1 * j + 0] = -elev_map.data[0].data[offset + i + stride1 * j + 0]
   map_cpy.info = elev_map.info
   map_cpy.layers = elev_map.layers
   map_cpy.basic_layers = elev_map.basic_layers
   map_cpy.outer_start_index = elev_map.outer_start_index
   map_cpy.inner_start_index = elev_map.inner_start_index
  # copy to tuple
   map_cpy.data[0].data = tuple(data_tmp)

def mapListener():
   #create node
   rospy.init_node('map_listener', anonymous=True)
   #subscribe /map topic
   rospy.Subscriber("/map_copy", msg.GridMap, callback)
   # spin() simply keeps python from exiting until this node is stopped
   pub = rospy.Publisher('reflected_map', msg.GridMap, queue_size=10)

   rate = rospy.Rate(1)  # 1hz
   while not rospy.is_shutdown():
       #publish copy of the map
       pub.publish(map_cpy)
       rate.sleep()

if __name__ == '__main__':
   mapListener()

