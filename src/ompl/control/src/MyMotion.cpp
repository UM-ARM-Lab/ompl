//
// Created by pmitrano on 2/19/20.
//

#include "ompl/control/MyMotion.h"
#include "ompl/control/SpaceInformation.h"

ompl::control::MyMotion::MyMotion(const SpaceInformation *si)
        : state(si->allocState()), control(si->allocControl())
{
}

const ompl::base::State *ompl::control::MyMotion::getState() const
{
    return state;
}

const ompl::control::MyMotion *ompl::control::MyMotion::getParentMotion() const
{
    return parent;
}
