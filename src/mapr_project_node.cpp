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
#include <std_msgs/UInt8.h>

using namespace std;
using namespace ros;

grid_map_msgs::GridMap globalMap;
std_msgs::UInt8 start_x;
std_msgs::UInt8 start_y;
std_msgs::UInt8 end_x;
std_msgs::UInt8 end_y;
int licznik = 0;

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

void pointCallback_startX(const std_msgs::UInt8& startX)
{
  ROS_INFO("Planner received start point X");
          start_x = startX;
//  std::cout << "Start X:" << double(start_x.data) << "\n";
}

void pointCallback_startY(const std_msgs::UInt8& startY)
{
  ROS_INFO("Planner received start point Y");
          start_y = startY;
// std::cout << "Start Y:" << double(start_y.data) << "\n";

}

void pointCallback_endX(const std_msgs::UInt8& endX)
{
  ROS_INFO("Planner received end point X");
          end_x = endX;
//          std::cout << "End X:" << double(end_x.data) << "\n";

}

void pointCallback_endY(const std_msgs::UInt8& endY)
{
  ROS_INFO("Planner received end point Y");
          end_y= endY;
//          std::cout << "End Y:" << double(end_y.data) << "\n";
}

int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");
    mapr_project::Planner2D planner_(nodeHandle);

    // setup the ROS loop rate
    ros::Rate loop_rate(0.5);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("/planned_path", 1000);

    // elevation map subscriber
    ros::Subscriber map_sub = nodeHandle.subscribe("/map_copy", 10, mapCallback);  
    ros::Subscriber pstart_x = nodeHandle.subscribe("/start_point_x", 10, pointCallback_startX);
    ros::Subscriber pstart_y = nodeHandle.subscribe("/start_point_y", 10, pointCallback_startY);
    ros::Subscriber pend_x = nodeHandle.subscribe("/end_point_x", 10, pointCallback_endX);
    ros::Subscriber pend_y = nodeHandle.subscribe("/end_point_y", 10, pointCallback_endY);

    while (ros::ok()){
        nav_msgs::Path plannedPath;
        planner_.returnPoints(start_x, start_y, end_x, end_y);
        plannedPath = planner_.planPath(globalMap);
	licznik++;
	std::cout << "Liczba wykonaych iteracji: " << licznik<< "\n";
        // publish the planned path
        path_pub.publish(plannedPath);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
