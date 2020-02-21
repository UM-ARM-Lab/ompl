#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

from math import sin, cos
from functools import partial
import matplotlib.pyplot as plt

# if the ompl module is not in the PYTHONPATH assume it is installed in a
# subdirectory of the parent directory called "py-bindings."
from os.path import abspath, dirname, join
import sys

sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
from ompl import base as ob
from ompl import control as oc


def isStateValid(si, state):
    return si.satisfiesBounds(state)


def motionsValid(si, motions):
    length = 0
    for motion in motions:
        if motion.getParentMotion() is not None:
            d = si.distance(motion.getState(), motion.getParentMotion().getState())
            length += d
    return length < 4.0


def propagate(start, control, duration, state):
    state.setX(start.getX() + control[0] * duration * cos(start.getYaw()))
    state.setY(start.getY() + control[0] * duration * sin(start.getYaw()))
    state.setYaw(start.getYaw() + control[1] * duration)


def plan():
    # construct the state space we are planning in
    space = ob.SE2StateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-10)
    bounds.setHigh(10)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-10)
    cbounds.setHigh(10)
    cspace.setBounds(cbounds)

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    svc = ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation()))
    ss.setStateValidityChecker(svc)

    ss.setStatePropagator(oc.StatePropagatorFn(propagate))

    mvc = oc.MotionsValidityCheckerFn(partial(motionsValid, ss.getSpaceInformation()))

    ss.setMotionsValidityChecker(mvc)

    # create a start state
    start = ob.State(space)
    start().setX(0.0)
    start().setY(0.0)
    start().setYaw(0.0)

    # create a goal state
    goal = ob.State(space)
    goal().setX(1.0)
    goal().setY(0.0)
    goal().setYaw(0.0)

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.1)

    # (optionally) set planner
    si = ss.getSpaceInformation()
    planner = oc.RRT(si)
    planner.setIntermediateStates(True)
    ss.setPlanner(planner)
    # (optionally) set propagation step size
    si.setPropagationStepSize(.1)
    si.setMinMaxControlDuration(1, 10)

    # attempt to solve the problem
    solved = ss.solve(10.0)

    print(solved.asString())
    if solved:
        # print the path to screen
        # print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())
        path = ss.getSolutionPath()
        xs = []
        ys = []
        length = 0
        for i in range(path.getStateCount() - 1):
            state = path.getState(i)
            next_state = path.getState(i+1)
            xs.append(state.getX())
            ys.append(state.getY())
            length += space.distance(state, next_state)

        plt.title("length: {}".format(length))
        plt.plot(xs, ys)
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    plan()
