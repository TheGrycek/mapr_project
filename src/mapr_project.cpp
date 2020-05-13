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

Planner2D::Planner2D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("Controlling UR node started.");
    configure();
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
     if (coordX->values[0]<5.1&&coordX->values[0]>5.0){
        if (coordY->values[0]<4.0&&coordY->values[0]>-5.0){
             return false;
         }
     }

    //! Comment this part of the code if you'd like to use occupancy grid

    //! Your code goes below
    // Hint: uncoment the code below:

     //int row, col;
     //float originX = occupancyMap.info.origin.position.x;
     //float originY = occupancyMap.info.origin.position.y;
     //float resolution = occupancyMap.info.resolution;

       // col = ((coordX->values[0])-originX)/resolution;

       //row = ((coordY->values[0])-originY)/resolution;


/*    if (occupancyMap.data.size())
     {
         for (int i = -5; i < 5; i++)
                {
                    for (int j = -5; j < 5; j++)
                    {
                        if (int(occupancyMap.data[(col+i) + (row+j) * occupancyMap.info.width])>0)
                        {
                             return false;
                        }
                    }
                 }

     }
*/
    

    //! Your code goes below
    // Hint: uncoment the code below:

	
    std::cout << "gridMap.info.pose.position.x " << gridMap.info.pose.position.x << "\n";
    std::cout << "gridMap.info.pose.position.y " << gridMap.info.pose.position.y << "\n";
    std::cout << "gridMap.info.pose.position.z " << gridMap.info.pose.position.z << "\n";
    std::cout << "gridMap.info.resolution " << gridMap.info.resolution << "\n";
    std::cout << "gridMap.info.length_x " << gridMap.info.length_x << "\n";
    std::cout << "gridMap.info.length_y " << gridMap.info.length_y << "\n";
	/*
    if(gridMap.info.length_x )
	{
      	   //std::cout << "gridMap.info.length_x -  " << gridMap.info.length_x << "\n";
	}
	*/

    //! Your code goes above
    return true;
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
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[0];
        poseMsg.pose.position.z = 0.01;
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

/*!
 * plan path
 */
nav_msgs::Path Planner2D::planPath(const grid_map_msgs::GridMap& globalMap){
    gridMap = globalMap;
	
//nav_msgs::Path Planner2D::planPath(const nav_msgs::OccupancyGrid& globalMap){
    //occupancyMap = globalMap;
	
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
void Planner2D::configure(void){
    dim = 2;//2D problem
    maxStepLength = 0.1;// max step length

    // create bounds for the x axis
    coordXBound.reset(new ob::RealVectorBounds(dim-1));
    coordXBound->setLow(-1.0);
    coordXBound->setHigh(13.0);

    // create bounds for the y axis
    coordYBound.reset(new ob::RealVectorBounds(dim-1));
    coordYBound->setLow(-5.0);
    coordYBound->setHigh(5.0);

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
    (*start.get())[0]=0.0;
    (*start.get())[1]=-2.5;
//    start.get()->random();

    // define the goal position
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]=12.0;
    (*goal.get())[1]=-4.0;
//    goal.get()->random();
}

} /* namespace */
