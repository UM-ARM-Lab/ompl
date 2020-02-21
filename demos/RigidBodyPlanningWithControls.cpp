/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
#include <iostream>
#include <numeric>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/cem/CEM.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "matplotlibcpp.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace plt = matplotlibcpp;

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}

bool motionsValid(const oc::SpaceInformation *si, oc::MyMotions const motions)
{
    auto const length = std::accumulate(motions.cbegin(),
                                        motions.cend(),
                                        0.0,
                                        [&](double acc, oc::MyMotion *motion)
                                        {
                                            if (motion->parent == nullptr)
                                            {
                                                return acc;
                                            }
                                            return acc + si->distance(motion->state, motion->parent->state);
                                        });
    return length < 2.0;
}


void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double *ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
            pos[0] + ctrl[0] * duration * cos(rot),
            pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
            rot + ctrl[1] * duration);
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-10);
    cbounds.setHigh(10);
    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // set state validity checking for this space
    si->setStateValidityChecker([&si](const ob::State *state) { return isStateValid(si.get(), state); });

    // set state validity checking for this space
    si->setMotionsValidityChecker([&si](const oc::MyMotions motions) { return motionsValid(si.get(), motions); });

    // set the state propagation routine
    si->setStatePropagator(propagate);

    si->setMinMaxControlDuration(1, 10);
    si->setPropagationStepSize(0.1);

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(0.5);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);

    // for CEM only
    // pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    // create a planner for the defined space
    auto planner(std::make_shared<oc::RRT>(si));
    //auto planner(std::make_shared<oc::CEM>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    planner->setIntermediateStates(true);
//    planner->setIterations(5);
//    planner->setNumSamples(100);
//    planner->setTopK(10);
//    planner->setTimeSteps(5);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    // si->printSettings(std::cout);

    // print the problem settings
    // pdef->print(std::cout);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10000.0);

    std::vector<double> xs;
    std::vector<double> ys;
    std::cout << solved.asString() << '\n';
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        auto path = pdef->getSolutionPath()->as<ompl::control::PathControl>();

        auto length = 0.0;
        for (auto i{0u}; i < path->getStateCount(); ++i)
        {
            auto const abstract_state = path->getState(i);
            ompl::base::ScopedState<ompl::base::SE2StateSpace> state(space);
            state = abstract_state;
            xs.push_back(state->getX());
            ys.push_back(state->getY());

            if (i + 1 < path->getStateCount())
            {
                auto const next_abstract_state = path->getState(i + 1);
                length += space->distance(abstract_state, next_abstract_state);
            }
        }

        std::cout << length << '\n';

        plt::plot(xs, ys);
        plt::xlim(-10, 10);
        plt::ylim(-10, 10);
        plt::axis("equal");
        plt::show();
    } else
    {
        std::cout << "No solution found" << std::endl;
    }

}

int main(int /*argc*/, char ** /*argv*/)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
//    ompl::RNG::setSeed(0);

    plan();

    return 0;
}
