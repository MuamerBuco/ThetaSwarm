#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <Eigen/Dense>
#include <vector>

#include "autoMR.h"

// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
    PLANNER_AITSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

int makeTrajectory(FullStateTrajectory &out_trajectory, FullStateTrajectory const &waypoints, std::string plannerName);

// Temporary "collision checker", currently always works
class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si) :
        ompl::base::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const override
    {
        return this->clearance(state) > 0.0;
    }

    // TODO implement obstacle avoidance
    // Returns the distance from the given state's position to the
    // boundary of the obstacles/obstacle
    double clearance(const ompl::base::State* state) const override
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        // const auto* state2D =
        //     state->as<ompl::base::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        // double x = state2D->values[0];
        // double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        // return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;

        return 1;
    }
};

class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
    public:
        ClearanceObjective(const ompl::base::SpaceInformationPtr& si) :
            ompl::base::StateCostIntegralObjective(si, true)
        {
        }

        // Our requirement is to maximize path clearance from obstacles,
        // but we want to represent the objective as a path cost
        // minimization. Therefore, we set each state's cost to be the
        // reciprocal of its clearance, so that as state clearance
        // increases, the state cost decreases.
        ompl::base::Cost stateCost(const ompl::base::State* s) const override
        {
            return ompl::base::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
                std::numeric_limits<double>::min()));
        }
};

#endif //ROBOT_NAVIGATION_H