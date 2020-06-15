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

double point_start_x = -0.5;
double point_start_y = -0.5;
double point_end_x = -5.0;
double point_end_y = -5.0;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
 
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state);
    }

    double clearance(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();
        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[0];
        int col = (x)/(-0.1);
     	int row = (y)/(-0.1);

	float resolution = gridMap.info.resolution;

  	float point_start_z = 0;
	float point_end_z = 0;

  	if(gridMap.info.length_x>0)  // Mapa wysokosci subskyrbowana
	{
		// Wysokosc z jakiej startujemy i do jakiej zmierzamy
		point_start_z = gridMap.data[0].data[(point_start_x/resolution*(-1)) + (point_start_y/resolution*(-1)) * 64];
		point_end_z = gridMap.data[0].data[(point_end_x/resolution*(-1)) + (point_end_y/resolution*(-1)) * 64];
		//std::cout << "point_start_z: " << point_start_z << " point_end_z: " << point_end_z << "\n";

		// Idziemy z gorki
		if(point_start_z > point_end_z) 	
		{
			if ((gridMap.data[0].data[col + row * 64]<point_start_z) && (gridMap.data[0].data[col + row * 64]>point_end_z))
	    		{
				
				return gridMap.data[0].data[col + row * 64]; 
	 		} 
		 	else
			{
				return gridMap.data[0].data[col + row * 64]*gridMap.data[0].data[col + row * 64]; 
			}
		}
		else // Idziemy pod gore
		{
			if ((gridMap.data[0].data[col + row * 64]>point_start_z) && (gridMap.data[0].data[col + row * 64]<point_end_z))
	    		{
				return gridMap.data[0].data[col + row * 64]; 
	 		} 
		 	else
			{
				return gridMap.data[0].data[col + row * 64]*gridMap.data[0].data[col + row * 64]; 
			}

		}	
	}
	else // Brak mapy
	{
		return 1;
	}
    }
};

class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(si_->getStateValidityChecker()->clearance(s));
    }
};

//Optimization - getPathLengthObjective
 ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
 {
     return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
 }

//Optimization - getClearanceObjective
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);

}

//Optimization - getBalancedObjective
ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 1.0);
    opt->addObjective(clearObj, 10.0);
    return ob::OptimizationObjectivePtr(opt);
}



Planner2D::Planner2D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("Planner node started.");
    configure(point_start_x, point_start_y, point_end_x, point_end_y);
}

Planner2D::~Planner2D()
{
}



void Planner2D::returnPoints(std_msgs::UInt8 pStartX, std_msgs::UInt8 pStartY,
                             std_msgs::UInt8 pEndX, std_msgs::UInt8 pEndY){
    //point_start_x = double(pStartX.data) * (-0.1);
    //point_start_y = double(pStartY.data) * (-0.1);
    //point_end_x = double(pEndX.data) * (-0.1);
    //point_end_y = double(pEndY.data) * (-0.1);
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

	const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];
	// potrzebne do obliczania wspl. Z sciezki
	int col = (x)/(-0.1);
     	int row = (y)/(-0.1);

        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = x;
        poseMsg.pose.position.y = y;
	if (gridMap.info.length_x)
    	{
		poseMsg.pose.position.z = gridMap.data[0].data[col + row * 64] +0.1; 
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

    	gridMap = globalMap;
	configure(point_start_x, point_start_y, point_end_x, point_end_y);

	//std::cout << "point_start_x" << point_start_x << std::endl;
	//std::cout << "point_start_y" << point_start_y << std::endl;

   	 // Construct the robot state space in which we're planning. We're
	// planning in [0,1]x[0,1], a subset of R^2.
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

	// Set the bounds of space to be in [0,1].
	space->as<ob::RealVectorStateSpace>()->setBounds(-6.4, 0.0);

	// Construct a space information instance for this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

	 // Set the object used to check which states in the space are valid
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setup();

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = point_start_x;
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = point_start_y;

	// Set our robot's goal state to be the top-right corner of the
	// environment, or (5,5).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = point_end_x;
	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = point_end_y;

	// Create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// Set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	// Create the optimization objective
	pdef->setOptimizationObjective(getBalancedObjective(si));

	// Construct our optimizing planner using the RRTstar algorithm.
	auto optimizingPlanner(std::make_shared<og::RRTstar>(si));
	optimizingPlanner->setRange(maxStepLength);// max step length

	// Set the problem instance for our planner to solve
	optimizingPlanner->setProblemDefinition(pdef);
	optimizingPlanner->setup();

	// attempt to solve the planning problem within one second of
	// planning time
	ob::PlannerStatus solved = optimizingPlanner->ob::Planner::solve(1.0);

 	nav_msgs::Path plannedPath;
	 if (solved)
 	    {
  	    	plannedPath=extractPath(pdef.get());
 	    }
  	   else
	    {
  	       std::cout << "No solution found." << std::endl;
	    }

	 return plannedPath;
}

/// configure planner
void Planner2D::configure(double point_start_x, double point_start_y, double point_end_x, double point_end_y)
{
        maxStepLength = 0.05;// max step length
}

} /* namespace */
