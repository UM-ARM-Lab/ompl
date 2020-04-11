#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/util/RandomNumbers.h>

#include <random>
#include <memory>
#include <vector>
#include <functional>
#include <armadillo>
#include <ompl/tools/benchmark/Benchmark.h>

namespace ompl
{
    namespace control
    {

        class CEM : public base::Planner
        {
          public:
            using Controls = std::vector<Control *>;
            using Parameters = arma::vec;
            using ParametersConverter = std::function<Controls(Parameters)>;
            using ParametersInitializer = std::function<Parameters()>;
            using PropagateWhileValidFn = std::function<bool(base::State * , Control const * , base::State * ,
                                                             PathControl & , SpaceInformationPtr const)>;
            using DebugSampledPathFn = std::function<void(PathControl, float)>;
            using IterEndCallback = std::function<void(arma::gmm_full const &, double, unsigned int, long)>;

            tools::MetricsCallback on_metrics_fn_;
            ParametersConverter parameters_converter_;
            ParametersInitializer parameters_initializer_;
            DebugSampledPathFn debug_sampled_path_fn_;
            DebugSampledPathFn debug_fn_2_;
            PropagateWhileValidFn propagate_while_valid_fn_;
            IterEndCallback on_iter_end_fn_;
            PathCostFn cost_fn_;

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

            void setNPrimitives(unsigned int n_primitives);

            void setNMixtureComponents(unsigned int n_components);

            void setInitialVariance(std::vector<double> initial_variance);

            unsigned int iterations() const
            {
                return iterations_;
            }

            std::string iterationsProperty() const
            {
                return std::to_string(iterations());
            }

          private:
            unsigned int iterations_{1};
            unsigned int n_primitives_{1};
            unsigned int top_k_{10};
            std::vector<double> initial_variance_;
            unsigned int num_samples_{100};
            int n_mixture_components_{100};
            base::OptimizationObjectivePtr opt_;
            RNG rng_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformationPtr siC_;

            /** \brief dimension of control space */
            // this member MUST be declared after siC_
            int n_z_;
            arma::gmm_full gmm_model;
        };
    }
}


