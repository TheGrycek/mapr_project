/*
 * mapr_project.cpp
 *
 *  Created on: May 13, 2020
 *      Author: Kamil Miedzinski Mateusz Grycmacher
 *	 Institute: Instute of Home, Poznan University of Technology
 */

#include "../include/mapr_project/mapr_project.hpp"

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include <thread>

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace mapr_project {

grid_map_msgs::GridMap gridMap;
double point_start_x = 0.0;
double point_start_y = 0.0;
double point_end_x = 0.0;
double point_end_y = 0.0;

Planner2D::Planner2D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("Controlling UR node started.");
    configure(point_start_x, point_start_y, point_end_x, point_end_y);
}

Planner2D::~Planner2D()
{
}

/// check if the current state is valid
bool isStateValid(const ob::State *state){
    // get x coord of the robot
    int test = 0;
    const auto *coordX =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const auto *coordY =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    //! Comment this part of the code if you'd like to use occupancy grid
    //     define the obstacle
    // if (coordX->values[0]<-3.1&&coordX->values[0]>-3.2){
    //    if (coordY->values[0]<3.0&&coordY->values[0]>-2.0){
    //         return false;
    //     }
    // }

     float originX = gridMap.info.pose.position.x;
     float originY = gridMap.info.pose.position.y;
     float resolution = gridMap.info.resolution;

     int col = (coordX->values[0])*(-1)/resolution;
     int row = (coordY->values[0])*(-1)/resolution;
     int stride  = 60; 

     /*if (gridMap.info.length_x)
     {
 	//std::cout << "gridMap.data[0].data[col + row * stride] " << gridMap.data[0].data[col + row * stride] << "\n";
        if (gridMap.data[0].data[col + row * stride] > 3)
      	{
            return false;
       	}
	
     }*/

    //! Your code goes below
    // Hint: uncoment the code below:
	/*
    std::cout << "gridMap.info.pose.position.x " << gridMap.info.pose.position.x << "\n";
    std::cout << "gridMap.info.pose.position.y " << gridMap.info.pose.position.y << "\n";
    std::cout << "gridMap.info.pose.position.z " << gridMap.info.pose.position.z << "\n";
    std::cout << "gridMap.info.resolution " << gridMap.info.resolution << "\n";
    std::cout << "gridMap.info.length_x " << gridMap.info.length_x << "\n";
    std::cout << "gridMap.info.length_y " << gridMap.info.length_y << "\n";
	
    if(gridMap.info.length_x )
	{
      	   std::cout << "gridMap.data[0].layout.dim[0].stride " << gridMap.data[0].layout.dim[0].stride << "\n";
	   std::cout << "gridMap.data[0].layout.dim[1].stride " << gridMap.data[0].layout.dim[1].stride << "\n";
	   std::cout << "gridMap.data[0].layout.dim[0].size " << gridMap.data[0].layout.dim[0].size << "\n";
	   std::cout << "gridMap.data[0].layout.dim[1].size " << gridMap.data[0].layout.dim[1].size << "\n";
	   std::cout << "gridMap.data[0].layout.data_offset " << gridMap.data[0].layout.data_offset << "\n";
	   std::cout << "gridMap.data[0].data[1000] " << gridMap.data[0].data[1000] << "\n";
	}
	*/
    //! Your code goes above
    return true;
}


void Planner2D::returnPoints(std_msgs::UInt8 pStartX, std_msgs::UInt8 pStartY,
                             std_msgs::UInt8 pEndX, std_msgs::UInt8 pEndY){
    point_start_x = double(pStartX.data) * 0.1 - 6.0;
    point_start_y = double(pStartY.data) * 0.1 - 6.0;
    point_end_x = double(pEndX.data) * 0.1 - 6.0;
    point_end_y = double(pEndY.data) * 0.1 - 6.0;
}

/// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){
    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
    path->print(std::cout);
    // convert to geometric path
    const auto *path_ = path.get()->as<og::PathGeometric>();
    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // get x coord of the robot
        const auto *coordX =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        const auto *coordY =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

	// potrzebne do obliczania wspl. Z sciezki
	int col = ((coordX->values[0]))/(-0.1);
     	int row = ((coordY->values[0]))/(-0.1);

        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[0];
        if (gridMap.info.length_x)
    	{
		poseMsg.pose.position.z = gridMap.data[0].data[col + row * 60] +0.1; 
 	}
	else
	{
		poseMsg.pose.position.z = 0;
	} 
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/map";
        poseMsg.header.stamp = ros::Time::now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}


nav_msgs::Path Planner2D::planPath(const grid_map_msgs::GridMap& globalMap){

    configure(point_start_x, point_start_y, point_end_x, point_end_y);
    gridMap = globalMap;
	
    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    // define state checking callback
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001);

    // problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());

    // create planner
    auto planner(std::make_shared<og::RRTConnect>(si));
    // configure the planner
    planner->setRange(maxStepLength);// max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    // solve motion planning problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    nav_msgs::Path plannedPath;
    if (solved) {// if cussess
        // get the planned path
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
}

/// configure planner
void Planner2D::configure(double point_start_x, double point_start_y, double point_end_x, double point_end_y){

    dim = 2;//2D problem
    maxStepLength = 0.1;// max step length

    // create bounds for the x axis
    coordXBound.reset(new ob::RealVectorBounds(dim-1));
    coordXBound->setLow(-6.0);
    coordXBound->setHigh(0.0);

    // create bounds for the y axis
    coordYBound.reset(new ob::RealVectorBounds(dim-1));
    coordYBound->setLow(-6.0);
    coordYBound->setHigh(0.0);

    // construct the state space we are planning in
    auto coordX(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    auto coordY(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    space = coordX +coordY;

    // create bounds for the x axis
    coordX->setBounds(*coordXBound.get());

    // create bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    // define the start position
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0]= point_start_x;
    (*start.get())[1]= point_start_y;
//    start.get()->random();

    // define the goal position
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]= point_end_x;
    (*goal.get())[1]= point_end_y;
//    goal.get()->random();
}

} /* namespace */
