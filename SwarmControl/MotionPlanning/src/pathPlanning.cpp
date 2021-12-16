#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

// For std::make_shared
#include <memory>
#include <fstream>

#include "../../../util/util.h"

#include "../pathPlanning.h"

#define RUN_TIME 1.0
#define PLANNER_USED "BITstar"
#define PLANNING_OBJECTIVE "PathLength"
#define TRAJECTORY_FILE_PATH "path.txt"
#define LOG_LEVEL 1

#define MAX_YAW 360
#define MAX_X 190
#define MAX_Y 130

#define MIN_YAW 0
#define MIN_X 0
#define MIN_Y 0

using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// returns the path length objective
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

// returns the threashold path length objective
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

// returns the clearence length objective
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

// returns the balanced objective
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}

// returns the path length objective with cost to go objective
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

// set planner to be used
ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_AITSTAR:
        {
            return std::make_shared<og::AITstar>(si);
            break;
        }
        case PLANNER_BFMTSTAR:
        {
            return std::make_shared<og::BFMT>(si);
            break;
        }
        case PLANNER_BITSTAR:
        {
            return std::make_shared<og::BITstar>(si);
            break;
        }
        case PLANNER_CFOREST:
        {
            return std::make_shared<og::CForest>(si);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return std::make_shared<og::FMT>(si);
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            return std::make_shared<og::InformedRRTstar>(si);
            break;
        }
        case PLANNER_PRMSTAR:
        {
            return std::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            return std::make_shared<og::SORRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

// set objective to be used
ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_THRESHOLDPATHLENGTH:
            return getThresholdPathLengthObj(si);
            break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective1(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

// adjust the parameters
bool argParse(double *runTimePtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string plannerName)
{
    // Set the log-level
    unsigned int logLevel = LOG_LEVEL;

    // Switch to setting the log level:
    if (logLevel == 0u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }
    else if (logLevel == 1u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    }
    else if (logLevel == 2u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    }
    else
    {
        std::cout << "Invalid log-level integer." << std::endl;
        return false;
    }

    // Set the runtime as a double
    double runTime = RUN_TIME; 

    // Sanity check
    if (runTime <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl;
        return false;
    }

    // Map the string to the enum
    if ( plannerName == "AITstar" ) 
    {
        *plannerPtr = PLANNER_AITSTAR;
    }
    else if ( plannerName == "BFMTstar" )
    {
        *plannerPtr = PLANNER_BFMTSTAR;
    }
    else if ( plannerName == "BITstar" )
    {
        *plannerPtr = PLANNER_BITSTAR;
    }
    else if ( plannerName == "CForest" )
    {
        *plannerPtr = PLANNER_CFOREST;
    }
    else if ( plannerName == "FMTstar" )
    {
        *plannerPtr = PLANNER_FMTSTAR;
    }
    else if ( plannerName == "InformedRRTstar" )
    {
        *plannerPtr = PLANNER_INF_RRTSTAR;
    }
    else if ( plannerName == "PRMstar" )
    {
        *plannerPtr = PLANNER_PRMSTAR;
    }
    else if ( plannerName == "RRTstar" )
    {
        *plannerPtr = PLANNER_RRTSTAR;
    }
    else if ( plannerName == "SORRTstar" )
    {
        *plannerPtr = PLANNER_SORRTSTAR;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl;
        return false;
    }

    // Set the specified objective as a string
    std::string objectiveStr = PLANNING_OBJECTIVE;

    // Map the string to the enum
    if ( objectiveStr == "PathClearance" )
    {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    }
    else if ( objectiveStr == "PathLength" )
    {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    }
    else if ( objectiveStr == "ThresholdPathLength" )
    {
        *objectivePtr = OBJECTIVE_THRESHOLDPATHLENGTH;
    }
    else if ( objectiveStr == "WeightedLengthAndClearanceCombo" )
    {
        *objectivePtr = OBJECTIVE_WEIGHTEDCOMBO;
    }
    else
    {
        std::cout << "Invalid objective string." << std::endl;
        return false;
    }

    // Looks like we parsed the arguments successfully
    return true;
}

bool operator==(SingleStateTrajectory a, SingleStateTrajectory b)
{
    if(a.pose.x == b.pose.x && a.pose.y == b.pose.y && a.pose.yaw == b.pose.yaw && a.LED_state.program == b.LED_state.program 
    && a.bucket_state.extension == b.bucket_state.extension && a.bucket_state.tilt == b.bucket_state.tilt) return 1;
    else return 0;
    
}

// TODO1 assume that the given targets for YAW are [0, 360] degrees 
SingleStateTrajectory normalizeSingleStatePose(SingleStateTrajectory single_state)
{
    single_state.pose.x = MapValueToRange(MIN_X, -1, MAX_X, 1, single_state.pose.x);
    single_state.pose.y = MapValueToRange(MIN_Y, -1, MAX_Y, 1, single_state.pose.y);
    //single_state.pose.yaw = MapValueToRange(MIN_YAW, -3.14, MAX_YAW, 3.14, single_state.pose.yaw);

    return single_state;
}

// TODO1 assume that the given targets for YAW are [0, 360] degrees 
SingleStateTrajectory denormalizeSingleStatePose(SingleStateTrajectory single_state)
{
    single_state.pose.x = MapValueToRange(-1, MIN_X, 1, MAX_X, single_state.pose.x);
    single_state.pose.y = MapValueToRange(-1, MIN_Y, 1, MAX_Y, single_state.pose.y);
    //single_state.pose.yaw = MapValueToRange(-3.14, MIN_YAW, 3.14, MAX_YAW, single_state.pose.yaw);

    return single_state;
}

// takes in a vector holding waypoints, returns FullTrajectory holding full path
int makeTrajectory(FullStateTrajectory& out_trajectory, FullStateTrajectory const &new_setpoints, std::string plannerName = PLANNER_USED)
{
    // TODO2 input current position when starting from an unknown point
    SingleStateTrajectory new_single_setpoint;

    // initializes by default to 0
    SingleStateTrajectory current_state;
    FullStateTrajectory trajectory_holder;

    // Convert pose values to match the set bounds. aka transform coordinate space to trajectory space, if setpoints are empty returns 0 trajectory
    if( !new_setpoints.empty() )
    {
        for( auto setpoint : new_setpoints)
        {
            setpoint = normalizeSingleStatePose(setpoint);
        }
    }
    else {
        std::cout << "New setpoint list is empty " << std::endl;
        return -1;
    }
    
    double runTime;
    optimalPlanner plannerType;
    planningObjective objectiveType;

    // set the runtime, planner and objective(not neccessary rn, but will be later during expansion)
    argParse(&runTime, &plannerType, &objectiveType, plannerName);

    // push initial position as first point
    trajectory_holder.push_back(current_state);

    for(int i = 0; i < new_setpoints.size(); i++)
    {
        // set waypoint window
        new_single_setpoint = new_setpoints.at(i) ;

        // std::cout << "The setpoint x before normalizing: " << new_single_setpoint.pose.x << std::endl;
        // std::cout << "The setpoint y before normalizing: " << new_single_setpoint.pose.y << std::endl;

        SingleStateTrajectory previous_single_setpoint;

        // if its the first waypoint, set waypoint to current position
        if(i > 0)
        {
            previous_single_setpoint = new_setpoints.at(i-1);
        }
        else previous_single_setpoint = current_state;

        // Construct the robot state space in which we're planning.
        auto space(std::make_shared<ob::SE2StateSpace>());

        // set boundaries, always [-1,1] for all, mapped later to coordinate system
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-1);
        bounds.setHigh(1);

        // The bounds for yaw are [-3.14, 3.14] -> -pi, pi
        space->setBounds(bounds);

        // Construct a space information instance for this state space
        auto space_information(std::make_shared<ob::SpaceInformation>(space));

        // Set the object used to check which states in the space are valid
        space_information->setStateValidityChecker(std::make_shared<ValidityChecker>(space_information));

        // setup the space information for state space
        space_information->setup();

        // create start and goal
        ob::ScopedState<ob::SE2StateSpace> start(space);
        ob::ScopedState<ob::SE2StateSpace> goal(space);

        // Create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(space_information));

        // Convert pose values to match the set bounds. aka transform coordinate space to trajectory space
        previous_single_setpoint = normalizeSingleStatePose(previous_single_setpoint);
        new_single_setpoint = normalizeSingleStatePose(new_single_setpoint);

        // std::cout << "The setpoint x after normalizing: " << new_single_setpoint.pose.x << std::endl;
        // std::cout << "The setpoint y after normalizing: " << new_single_setpoint.pose.y << std::endl;

        // set previous waypoint as start
        start->setX(previous_single_setpoint.pose.x);
        start->setY(previous_single_setpoint.pose.y);
        start->setYaw(previous_single_setpoint.pose.yaw);

        // set next waypoint as goal
        goal->setX(new_single_setpoint.pose.x);
        goal->setY(new_single_setpoint.pose.y);
        goal->setYaw(new_single_setpoint.pose.yaw);

        // Create the optimization objective
        pdef->setOptimizationObjective(allocateObjective(space_information, objectiveType));

        // Construct the optimal planner
        ob::PlannerPtr optimizingPlanner = allocatePlanner(space_information, plannerType);

        // Set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // attempt to solve the planning problem in the given runtime
        ob::PlannerStatus solved = optimizingPlanner->solve(runTime);

        if(solved)
        {
            og::PathGeometricPtr path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            std::vector<ob::State*> tempStates = path->getStates();

            // holder for the actual values, function requires vector
            std::vector<double> reals;
            
            for (auto state : tempStates)
            {
                // take numerical values from space and save to holding vector
                space->copyToReals(reals, state);

                current_state.pose.x = reals.at(0);
                current_state.pose.y = reals.at(1);
                current_state.pose.yaw = reals.at(2);

                // the default values for action space, modified later if specified
                current_state.bucket_state.extension = 0;
                current_state.bucket_state.tilt = 0;

                current_state.LED_state.program = NONE;

                // vector.back has undefined behaviour if pointed at empty vector, so check first
                if( !trajectory_holder.empty() )
                {
                    // push values to full list if its different from the preceding point, to avoid duplicates
                    if( !(current_state == trajectory_holder.back()) )
                    {
                        trajectory_holder.push_back(current_state);
                    }
                }

                // std::cout << "The new point is: " << pointHolder << std::endl;
                // std::cout << "Now at pass: " << counter << std::endl;
                // std::cout << "the state place 0: " << reals.at(0) << std::endl;
                // std::cout << "the state place 1: " << reals.at(1) << std::endl;
                // std::cout << "the state place 2: " << reals.at(2) << std::endl;
            }
        }
        else {
            std::cout << "No trajectory found!" << std::endl;
            return 0; // returns the initial current state which is set to 0
        }
        // embed the bucket and LED commands in the appropriate spot in the final trajectory holder 1 check if set to right point
        trajectory_holder.back().bucket_state = new_setpoints.at(i).bucket_state;
        trajectory_holder.back().LED_state = new_setpoints.at(i).LED_state;
    }
    // Convert normalized values back to real values, trajectory space(-1,1)(-1,1)(-3.14,3.14) => coordinate space 
    for(auto single_state : trajectory_holder)
    {
        single_state = denormalizeSingleStatePose(single_state);
    }

    // if the trajectory is constructed successfuly return it, if not it will only contain a single 0 state so return that
    out_trajectory = trajectory_holder;
    return 1;
}
