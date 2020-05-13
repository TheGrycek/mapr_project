/*
 * mapr_project_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: Kamil Miedzinski Mateusz Grycmacher
 *	 Institute: Instute of Home, Poznan University of Technology
 */

#include <ros/ros.h>
#include "../include/mapr_project/mapr_project.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>


using namespace std;
using namespace ros;

grid_map_msgs::GridMap globalMap;

//nav_msgs::OccupancyGrid globalMap;

 //occupancy map callback
/*void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
    globalMap = *mapMsg;
}*/

void mapCallback(const grid_map_msgs::GridMap& message)
{
  ROS_INFO("Grid map visualization received a map (timestamp %f) for visualization.",
            message.info.header.stamp.toSec());
  globalMap = message;
}


int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");
    mapr_project::Planner2D planner_(nodeHandle);

    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    // occupancy map subscriber
    ros::Subscriber map_sub = nodeHandle.subscribe("/map_copy", 10, mapCallback);

    while (ros::ok()){
        nav_msgs::Path plannedPath;
        plannedPath = planner_.planPath(globalMap);

        // publish the planned path
        path_pub.publish(plannedPath);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
