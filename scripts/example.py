#!/usr/bin/env python
import rospy
import math
import numpy as np
from grid_map_msgs import msg # import occupancy grid data

map_cpy = msg.GridMap()

data_tmp = []
variance_tmp = []
noise = []
sinusoid = []
first = True

def terrain(elev_map):
   global first, data_tmp, noise, variance_tmp

   stride0 = elev_map.data[0].layout.dim[0].stride
   stride1 = elev_map.data[0].layout.dim[1].stride

   cols = elev_map.data[0].layout.dim[0].size
   rows = elev_map.data[0].layout.dim[1].size

   offset = elev_map.data[0].layout.data_offset
   map_cpy.data = elev_map.data

   data_tmp = list(map_cpy.data[0].data)
   variance_tmp = list(map_cpy.data[1].data)

   for i in range(len(data_tmp)):
      data_tmp[i] = np.float32(1)

   for i in range(len(variance_tmp)):
      variance_tmp[i] = np.float32(0.00009999)

   data_tmp2 = data_tmp

   data_tmp2 = np.array(data_tmp2)
   data_tmp2 = data_tmp2.reshape(rows, cols)
   data_tmp2 = np.array([[data_tmp2[j][i] for j in range(len(data_tmp2))] for i in range(len(data_tmp2[0]))])
   data_tmp2 = data_tmp2.reshape(1, stride0)
   data_tmp2 = data_tmp2[0]
   data_tmp2.tolist()

   sinusoid1 = np.linspace(0.0, 2.111, len(data_tmp))
   sinusoid2 = np.linspace(0.0, 3.833, len(data_tmp))
   sinusoid3 = np.linspace(0.0, 4.73, len(data_tmp))
   sinusoid4 = np.linspace(0.0, 30.78, len(data_tmp))
   sinusoid5 = np.linspace(0.0, 6.58, len(data_tmp))
   sinusoid6 = np.linspace(0.0, 16.119, len(data_tmp))
   sinusoid7 = np.linspace(0.0, 8.3, len(data_tmp))
   sinusoid8 = np.linspace(0.0, 10.97, len(data_tmp))
   sinusoid9 = np.linspace(0.0, 70.12, len(data_tmp))
   sinusoid10 = np.linspace(0.0, 20.87, len(data_tmp))
   sinusoid11 = np.linspace(0.0, 80.9, len(data_tmp))



   for i in range(len(sinusoid1)):
      sinusoid1[i] = math.sin(sinusoid1[i])
      sinusoid2[i] = math.sin(sinusoid2[i])
      sinusoid3[i] = math.cos(sinusoid3[i])
      sinusoid4[i] = math.sin(sinusoid4[i])
      sinusoid5[i] = math.cos(sinusoid5[i])
      sinusoid6[i] = math.sin(sinusoid6[i])
      sinusoid7[i] = math.sin(sinusoid7[i])
      sinusoid8[i] = math.cos(sinusoid8[i])
      sinusoid9[i] = math.sin(sinusoid9[i])
      sinusoid10[i] = math.sin(sinusoid10[i])
      sinusoid11[i] = math.sin(sinusoid11[i])

   if first:
      noise = np.random.normal(0, 2, len(data_tmp))
      data_tmp = np.float32(((data_tmp + noise)/100) + (sinusoid1 * 2) + (sinusoid3 * 0.9) - (sinusoid6 ) + (sinusoid8) - (sinusoid7 * 1.2) + (sinusoid9/10))
      data_tmp2 = np.float32(((data_tmp2 + noise)/100) + (sinusoid2 * 1.3) + (sinusoid4/2) - (sinusoid5) + (sinusoid8 * 1.4) + (sinusoid10/1.2) + (sinusoid11/10))

      data_tmp2 = np.array(data_tmp2)
      data_tmp2 = data_tmp2.reshape(rows, cols)
      data_tmp2 = np.array([[data_tmp2[j][i] for j in range(len(data_tmp2))] for i in range(len(data_tmp2[0]))])
      data_tmp2 = data_tmp2.reshape(1, stride0)
      data_tmp2 = data_tmp2[0]
      data_tmp2.tolist()

      data_tmp = (data_tmp * 0.7 + data_tmp2 * 0.9 + ((data_tmp - data_tmp2) * 0.3) + 5.0) * 0.35

      first = False

   return(data_tmp, variance_tmp)


def callback(elev_map):
   global first, data_tmp, variance_tmp

   map_cpy.info = elev_map.info
   map_cpy.layers = elev_map.layers
   map_cpy.basic_layers = elev_map.basic_layers
   map_cpy.outer_start_index = elev_map.outer_start_index
   map_cpy.inner_start_index = elev_map.inner_start_index

   if first:
      data_tmp, variance_tmp = terrain(elev_map)

   map_cpy.data[1].data = tuple(variance_tmp)
   map_cpy.data[0].data = tuple(data_tmp)

def mapListener():

   rospy.init_node('map_listener', anonymous=True)
   rospy.Subscriber("/elevation_mapping/elevation_map_raw", msg.GridMap, callback)
   pub = rospy.Publisher('/map_copy', msg.GridMap, queue_size=10)
   rate = rospy.Rate(1)  # 1hz

   while not rospy.is_shutdown():
       pub.publish(map_cpy)
       rate.sleep()

if __name__ == '__main__':
   mapListener()
