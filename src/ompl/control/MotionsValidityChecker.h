//
// Created by pmitrano on 2/19/20.
//

#ifndef OMPL_PATHVALIDITYCHECKER_
#define OMPL_PATHVALIDITYCHECKER_


#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        OMPL_CLASS_FORWARD(Motion);
        using Motions = std::vector<Motion *>;

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateValidityChecker */
        OMPL_CLASS_FORWARD(MotionsValidityChecker);
        /// @endcond

        class MotionsValidityChecker
        {
            public:
            /** \brief Constructor */
            MotionsValidityChecker(SpaceInformation *si) : si_(si)
            {
            }

            /** \brief Constructor */
            MotionsValidityChecker(const SpaceInformationPtr &si) : si_(si.get())
            {
            }

            virtual ~MotionsValidityChecker() = default;

            virtual bool isValid(Motions motions) const = 0;

            protected:
            /** \brief The instance of space information this motions validity checker operates on */
            SpaceInformation *si_;
        };

        /** \brief The simplest motions validity checker: all motions are valid */
        class AllValidMotionsValidityChecker : public MotionsValidityChecker
        {
            public:
            /** \brief Constructor */
            AllValidMotionsValidityChecker(SpaceInformation *si) : MotionsValidityChecker(si)
            {
            }

            /** \brief Constructor */
            AllValidMotionsValidityChecker(const SpaceInformationPtr &si) : MotionsValidityChecker(si)
            {
            }

            /** \brief Always return true (all motions are considered valid) */
            bool isValid(Motions /*motions*/) const override
            {
                return true;
            }
        };

    }
}
#endif //OMPL_PATHVALIDITYCHECKER_
