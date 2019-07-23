#pragma once

#include "ompl/base/Planner.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/control/SpaceInformation.h"

#include <random>
#include <memory>
#include <vector>

struct Gaussian
{
    double mean{0.0};
    double stddev{1.0};
};

namespace ompl
{
    namespace control
    {
        using MyControls = std::vector<double>;

        class CEM : public base::Planner
        {
        public:
            explicit CEM(const base::SpaceInformationPtr &si);

            ~CEM() override = default;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

            unsigned int getNumSamples() const;

            void setNumSamples(unsigned int num_samples);

            unsigned int getTopK() const;

            void setTopK(unsigned int top_k);

            unsigned int getIterations() const;

            void setIterations(unsigned int iterations);

            unsigned int getTimeSteps() const;

            void setTimeSteps(unsigned int time_steps);

            unsigned int iterations() const
            {
                return iterations_;
            }


            base::Cost bestCost() const
            {
                return bestCost_;
            }

            std::string iterationsProperty() const
            {
                return std::to_string(iterations());
            }

            std::string bestCostProperty() const
            {
                return std::to_string(bestCost().value());
            }

        private:
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
            unsigned int iterations_{1};
            unsigned int time_steps_{1};
            unsigned int top_k_{10};
            unsigned int num_samples_{100};
            base::OptimizationObjectivePtr opt_;
            // std::gamma_distribution<> steps_distribution(1.0);
            std::vector<Gaussian> gaussians_;
            RNG rng_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformationPtr siC_;

            /** \brief dimension of control space */
            // this member MUST be declared after siC_
            unsigned int control_dim_;
        };
    }
}


