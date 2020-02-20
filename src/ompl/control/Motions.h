//
// Created by pmitrano on 2/19/20.
//

#ifndef OMPL_MOTIONS_
#define OMPL_MOTIONS_

#include "ompl/base/State.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace control
    {
        OMPL_CLASS_FORWARD(Motion);
        using Motions = std::vector<Motion *>;

        /** \brief Representation of a motion

        This only contains pointers to parent motions as we
        only need to go backwards in the tree. */
        class Motion
        {
            public:
            Motion() = default;

            /** \brief Constructor that allocates memory for the state and the control */
            Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
            {
            }

            ~Motion() = default;

            /** \brief The state contained by the motion */
            base::State *state{nullptr};

            /** \brief The control contained by the motion */
            Control *control{nullptr};

            /** \brief The number of steps the control is applied for */
            unsigned int steps{0};

            /** \brief The parent motion in the exploration tree */
            Motion *parent{nullptr};
        };
    }
}

#endif //OMPL_MOTIONS_
