//
// Created by pmitrano on 2/19/20.
//

#ifndef OMPL_MYMOTION_
#define OMPL_MYMOTION_

#include <vector>
#include "ompl/base/State.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace control
    {
        OMPL_CLASS_FORWARD(SpaceInformation);

        OMPL_CLASS_FORWARD(MyMotion);
        using MyMotions = std::vector<MyMotion *>;

        /** \brief Representation of a motion

        This only contains pointers to parent motions as we
        only need to go backwards in the tree. */
        class MyMotion
        {
            public:
            MyMotion() = default;

            /** \brief Constructor that allocates memory for the state and the control */
            MyMotion(const SpaceInformation *si);

            ~MyMotion() = default;

            base::State const *getState() const;

            control::MyMotion const *getParentMotion() const;

            Control const *getControl() const;

            /** \brief The state contained by the motion. This is the state that control brought us to */
            base::State *state{nullptr};

            /** \brief The control contained by the motion. This is the control that took use from parent to state */
            Control *control{nullptr};

            /** \brief The number of steps the control is applied for */
            unsigned int steps{0};

            /** \brief The parent motion in the exploration tree */
            MyMotion *parent{nullptr};
        };
    }
}

#endif //OMPL_MYMOTION_
