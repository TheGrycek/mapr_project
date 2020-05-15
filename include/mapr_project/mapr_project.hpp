/*
 * mapr_project.hpp
 *
 *  Created on: May 13, 2020
 *      Author: Kamil Miedzinski Mateusz Grycmacher
 *	 Institute: Instute of Home, Poznan University of Technology
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/UInt8.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Cost.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace mapr_project {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    Planner2D(ros::NodeHandle& _nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
//    nav_msgs::Path planPath(const grid_map_msgs::GridMap& globalMap);
   // nav_msgs::Path planPath(const nav_msgs::OccupancyGrid& globalMap);
    nav_msgs::Path planPath(const grid_map_msgs::GridMap& globalMap);

    void returnPoints(std_msgs::UInt8 pStartX, std_msgs::UInt8 pStartY,
                      std_msgs::UInt8 pEndX, std_msgs::UInt8 pEndY);

private:
    /// node handle
    ros::NodeHandle& nodeHandle;

    /// problem dim
    int dim;

    /// max step length
    double maxStepLength;

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::StateSpace> space;

    /// configure node
    void configure(double point_start_x, double point_start_y, double point_end_x,double point_end_y);

    /// extract path
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
