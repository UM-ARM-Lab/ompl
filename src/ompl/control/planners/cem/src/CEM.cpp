#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/Planner.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/ControlSpaceTypes.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/tools/config/SelfConfig.h"

#include "ompl/control/planners/cem/CEM.h"

#include <queue>
#include <numeric>


ompl::control::CEM::CEM(const ompl::base::SpaceInformationPtr &si) :
        ompl::base::Planner(si, "CEM"),
        siC_(std::static_pointer_cast<SpaceInformation>(si)),
        control_dim_(siC_->getControlSpace()->getDimension())
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.directed = true;

    Planner::declareParam<unsigned int>("iterations", this, &ompl::control::CEM::setIterations,
                                        &ompl::control::CEM::getIterations, "1:1:1000");
    Planner::declareParam<unsigned int>("time_steps", this, &ompl::control::CEM::setTimeSteps,
                                        &ompl::control::CEM::getTimeSteps, "1:1:1000");
    Planner::declareParam<unsigned int>("n_samples", this, &ompl::control::CEM::setNumSamples,
                                        &ompl::control::CEM::getNumSamples, "1:1:1000");
    Planner::declareParam<unsigned int>("top_K", this, &ompl::control::CEM::setTopK, &ompl::control::CEM::getTopK,
                                        "1:1:1000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return iterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

// optional, if additional setup/configuration is needed, the setup() method can be implemented
void ompl::control::CEM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
    } else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Initialize the control sampling distribution.
    gaussians_.resize(time_steps_ * control_dim_);
}


ompl::base::PlannerStatus ompl::control::CEM::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // get input states with PlannerInputStates helper, pis_
    const ompl::base::State *start = pis_.nextStart();
    if (pis_.haveMoreStartStates())
    {
        OMPL_ERROR("%s: CEM only supports one start state!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    // get the goal
    const ompl::base::Goal *goal = pdef_->getGoal().get();

    // if it does, terminate planning.
    ompl::base::Cost lowest_cost{opt_->infiniteCost()};
    auto state = si_->allocState();
    auto resulting_state{si_->allocState()};
    auto u = siC_->allocControl()->as<ompl::control::RealVectorControlSpace::ControlType>();

    struct CEMSolution
    {
        base::Cost cost_;
        MyControls controls_;

    public:
        CEMSolution(base::Cost cost, MyControls controls) : cost_(cost), controls_(controls)
        {}

        CEMSolution() = default;

        bool operator<(const CEMSolution &other) const
        {
            // TODO: use the opt_->costIsBetterThan somehow
            return cost_.value() < other.cost_.value();
        }
    };

    std::vector<CEMSolution> solutions;
    auto best_path = std::make_shared<ompl::control::PathControl>(siC_);
    for (auto iter{0u}; iter < iterations_; ++iter)
    {
        //TODO: don't ignore PTC

        // start state
        si_->copyState(state, start);

        for (auto j{0u}; j < num_samples_; ++j)
        {
            // Sample controls and evaluate the cost
            ompl::base::Cost accumulated_cost{0};
            ompl::control::PathControl path{siC_};
            MyControls controls;
            for (auto t{0u}; t < time_steps_; ++t)
            {
                // sample one control and a duration for this time step
                auto ui{0u};
                for (; ui < control_dim_; ++ui)
                {
                    double sample = rng_.gaussian(gaussians_[t * control_dim_ + ui].mean,
                                                  gaussians_[t * control_dim_ + ui].stddev);
                    (*u)[ui] = sample;
                    controls.emplace_back(sample);
                }

                // TODO: sample this also? Can't use gaussian because it's negative, but idk how to re-fit a gamma dist
                auto const steps = 1;

                // Try out the sampled actions and accumulate cost
                auto cost = opt_->costToGo(state, goal);
                accumulated_cost = opt_->combineCosts(accumulated_cost, cost);
                auto const valid_steps = siC_->propagateWhileValid(state, u, steps, resulting_state);
                path.append(state, u, valid_steps * siC_->getPropagationStepSize());

                si_->copyState(state, resulting_state);
            }

            // don't forget to append the final state
            path.append(resulting_state);

            // insert into the priority queue
            solutions.emplace_back(CEMSolution{accumulated_cost, controls});

            // keep it if it's the best so far
            if (opt_->isCostBetterThan(accumulated_cost, lowest_cost))
            {
                OMPL_INFORM("new best path: %f", accumulated_cost);
                lowest_cost = accumulated_cost;
                best_path = std::make_shared<ompl::control::PathControl>(path);
            }
        }

        // pick the top K sample and refit the guassians
        sort(solutions.begin(), solutions.end());

        // make the first axis the control dimensions, second axis top_k samples
        std::vector<std::vector<double>> controls_by_dimension(control_dim_ * time_steps_);
        for (auto it = solutions.cbegin(); it != solutions.cbegin() + top_k_; ++it)
        {
            for (auto dim{0u}; dim < control_dim_ * time_steps_; ++dim)
            {
                auto const x = it->controls_[dim];
                controls_by_dimension[dim].push_back(x);
            }
        }

        // compute the new mean and standard deviation along each axis
        for (auto dim{0u}; dim < control_dim_ * time_steps_; ++dim)
        {
            auto const &top_k_controls_for_dim = controls_by_dimension[dim];
            auto const mean =
                    std::accumulate(top_k_controls_for_dim.cbegin(), top_k_controls_for_dim.cend(), 0.0) / top_k_;
            auto varf = [&](double const sum, double x)
            {
                return sum + std::pow(x - mean, 2) / top_k_;
            };
            auto const var = std::accumulate(top_k_controls_for_dim.cbegin(), top_k_controls_for_dim.cend(), 0.0, varf);
            auto const stdev = std::sqrt(var);
            gaussians_[dim].mean = mean;
            gaussians_[dim].stddev = stdev;
        }
    }

    ompl::base::PlannerSolution psol(best_path);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, lowest_cost, true);

    pdef_->addSolutionPath(psol);

    // are we freeing memory that is owned by the psol?
    si_->freeState(state);
    si_->freeState(resulting_state);
    siC_->freeControl(u);

    return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
}

void ompl::control::CEM::clear()
{
    Planner::clear();
    // clear the data structures here
}

void ompl::control::CEM::getPlannerData(ompl::base::PlannerData &data) const
{
    // fill data with the states and edges that were created
    // in the exploration data structure
    // perhaps also fill control::PlannerData
}

unsigned int ompl::control::CEM::getTopK() const
{
    return top_k_;
}

unsigned int ompl::control::CEM::getTimeSteps() const
{
    return time_steps_;
}

void ompl::control::CEM::setTopK(unsigned int top_k)
{
    top_k_ = top_k;
}

void ompl::control::CEM::setTimeSteps(unsigned int time_steps)
{
    time_steps_ = time_steps;

    // this changes the dimensionality of our sampling distribution,
    // so we must construct new means/stddevs.
    gaussians_.resize(time_steps_ * control_dim_);
}

unsigned int ompl::control::CEM::getIterations() const
{
    return iterations_;
}

void ompl::control::CEM::setIterations(unsigned int iterations)
{
    iterations_ = iterations;
}

void ompl::control::CEM::setNumSamples(unsigned int num_samples)
{
    num_samples_ = num_samples;
}

unsigned int ompl::control::CEM::getNumSamples() const
{
    return num_samples_;
}
