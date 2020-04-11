#include <limits>
#include <ompl/base/Cost.h>

#include "ompl/base/Planner.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/tools/config/SelfConfig.h"

#include "ompl/control/planners/cem/CEM.h"

ompl::control::CEM::CEM(const base::SpaceInformationPtr &si) :
        base::Planner(si, "CEM"),
        siC_(std::static_pointer_cast<SpaceInformation>(si)),
        n_z_(static_cast<int>(siC_->getControlSpace()->getDimension()))
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.directed = true;

    Planner::declareParam<unsigned int>("iterations", this, &ompl::control::CEM::setIterations,
                                        &ompl::control::CEM::getIterations, "1:1:1000");
    Planner::declareParam<unsigned int>("time_steps", this, &ompl::control::CEM::setNPrimitives,
                                        &ompl::control::CEM::getTimeSteps, "1:1:1000");
    Planner::declareParam<unsigned int>("n_samples", this, &ompl::control::CEM::setNumSamples,
                                        &ompl::control::CEM::getNumSamples, "1:1:1000");
    Planner::declareParam<unsigned int>("top_K", this, &ompl::control::CEM::setTopK, &ompl::control::CEM::getTopK,
                                        "1:1:1000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return iterationsProperty(); });
}

// optional, if additional setup/configuration is needed, the setup() method can be implemented
void ompl::control::CEM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
}


ompl::base::PlannerStatus ompl::control::CEM::solve(const base::PlannerTerminationCondition &ptc)
{
    auto const start_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    checkValidity();

    auto const real_control_space = siC_->getControlSpace()->as<RealVectorControlSpace>();

    const base::State *start = pis_.nextStart();
    if (pis_.haveMoreStartStates())
    {
        OMPL_ERROR("%s: CEM only supports one start state!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    auto const *goal = pdef_->getGoal()->as<base::GoalRegion>();

    auto best_path = std::make_shared<ompl::control::PathControl>(siC_);
    arma::uword params_dim = n_primitives_ * n_z_;

    auto const initial_parameters = parameters_initializer_();
    gmm_model.reset(params_dim, n_mixture_components_);
    gmm_model.set_means(initial_parameters);
    if (initial_variance_.size() != static_cast<size_t>(n_z_))
    {
        throw std::runtime_error("initial variance must have one variance per state dimension");
    }
    arma::vec cov_diag = arma::ones(params_dim);
    for (auto i{0u}; i < params_dim; ++i)
    {
        auto const j = i % n_z_;
        cov_diag(i) = initial_variance_[j];
    }
    arma::mat const initial_cov = arma::diagmat(cov_diag);
    arma::cube initial_covs = arma::zeros(params_dim, params_dim, n_mixture_components_);

    for (auto i{0}; i < n_mixture_components_; ++i)
    {
        initial_covs.slice(i) = initial_cov;
    }
    gmm_model.set_fcovs(initial_covs);

    double best_cost{std::numeric_limits<double>::max()};
    for (auto iter{0u}; iter < iterations_; ++iter)
    {
        if (ptc)
        {
            OMPL_INFORM("Exiting based on termination criterion");
            break;
        }

        // repeatedly sample until we get enough valid paths
        auto j{0u};
        auto attempts{0u};
        arma::vec costs(num_samples_);
        arma::mat all_params(params_dim, num_samples_, arma::fill::zeros);
        std::vector<PathControl> paths;
        while (true)
        {
            ++attempts;
            Parameters const params_j = gmm_model.generate();

            // Sample controls and evaluate the cost

            auto const controls_z_j = parameters_converter_(params_j);

            auto path_is_valid = true;
            ompl::control::PathControl path{siC_};
            auto *final_state = si_->allocState();
            auto *state = si_->allocState();
            si_->copyState(state, start);
            base::Cost accumulated_cost{0};
            for (auto t{0u}; t < controls_z_j.size(); ++t)
            {
                auto const control = controls_z_j[t];

                if (!real_control_space->satisfiesBounds(control))
                {
                    path_is_valid = false;
                    break;
                }

                auto const valid = propagate_while_valid_fn_(state, control, final_state, path, siC_);
                if (!valid)
                {
                    path_is_valid = false;
                    break;
                }

                auto const motion_cost = opt_->motionCost(state, final_state);
                opt_->combineCosts(accumulated_cost, motion_cost);

                siC_->copyState(state, final_state);
            }
            si_->freeState(state);

            if (!path_is_valid)
            {
                continue;
            }

            si_->freeState(final_state);
            auto const path_cost = cost_fn_(path, goal, siC_.get());

            debug_sampled_path_fn_(path, path_cost);

            paths.emplace_back(path);
            OMPL_INFORM("feasible sample %d/%d found with cost %f on attempt %d",
                        j,
                        num_samples_,
                        path_cost,
                        attempts);

            costs(j) = path_cost;
            all_params.col(j) = params_j;

            ++j;

            if (j == num_samples_)
            {
                break;
            }
        }

        // pick the top K sample and refit the distribution
        arma::uvec const sorted_indices = arma::sort_index(costs);
        arma::uvec const top_k_indices = sorted_indices(arma::span(0, top_k_ - 1));
        auto const top_k_params = all_params.cols(top_k_indices);
        auto const km_iter = 4;
        auto const em_iter = 10;
        auto const var_floor = 1e-10;
        auto const verbose = false;
        bool success = gmm_model.learn(top_k_params,
                                       n_mixture_components_,
                                       arma::maha_dist,
                                       arma::random_subset,
                                       km_iter,
                                       em_iter,
                                       var_floor,
                                       verbose);


        for (auto i{0u}; i < top_k_; ++i)
        {
            auto const path = paths.at(sorted_indices(i));
            auto const path_cost = costs(sorted_indices(i));
            debug_fn_2_(path, path_cost);
        }

        best_path = std::make_shared<ompl::control::PathControl>(paths.at(sorted_indices(0)));
        auto const new_lowest_cost = costs(sorted_indices(0));

        // add noise to the covariances, as the paper suggests. To reduce local minima
        auto fcovs = arma::cube(gmm_model.fcovs.begin(), params_dim, params_dim, n_mixture_components_);
        for (auto i{0}; i < n_mixture_components_; ++i)
        {
            fcovs.slice(i) += 1e-4 * initial_covs.slice(i);
        }
        gmm_model.set_fcovs(fcovs);

        if (not success)
        {
            OMPL_ERROR("Error fitting GMM");
        }

        auto const now = std::chrono::high_resolution_clock::now().time_since_epoch();
        auto const time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        on_iter_end_fn_(gmm_model, new_lowest_cost, iter, time_ms);

        if (new_lowest_cost < best_cost)
        {
            best_cost = new_lowest_cost;
            OMPL_INFORM("new best cost %f at time %6d", best_cost, time_ms);
            tools::Metrics other_metrics;
            tools::Metric means_metric{};
            means_metric.dims.emplace_back(gmm_model.means.size());
            means_metric.value = arma::conv_to<std::vector<double>>::from(gmm_model.means);
            means_metric.name = "mean";
            tools::Metric fcovs_metric{};
            fcovs_metric.dims.emplace_back(gmm_model.fcovs.n_slices);
            fcovs_metric.dims.emplace_back(gmm_model.fcovs.n_rows);
            fcovs_metric.dims.emplace_back(gmm_model.fcovs.n_cols);
            fcovs_metric.value = arma::conv_to<std::vector<double>>::from(arma::vectorise(gmm_model.fcovs));
            fcovs_metric.name = "fcovs";
            other_metrics.push_back(means_metric);
            other_metrics.push_back(fcovs_metric);
            on_metrics_fn_(time_ms, best_cost, other_metrics);
        }
    }

    base::PlannerSolution psol(best_path);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, base::Cost{best_cost}, true);

    pdef_->addSolutionPath(psol);

    // FIXME: are we freeing memory correctly, for state/resulting state, or path? or psol?
    return base::PlannerStatus::APPROXIMATE_SOLUTION;
}

void ompl::control::CEM::clear()
{
    Planner::clear();
    // clear the data structures here
}

void ompl::control::CEM::getPlannerData(base::PlannerData &data) const
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
    return n_primitives_;
}

void ompl::control::CEM::setTopK(unsigned int top_k)
{
    top_k_ = top_k;
}

void ompl::control::CEM::setNPrimitives(unsigned int n_primitives)
{
    n_primitives_ = n_primitives;
}

void ompl::control::CEM::setNMixtureComponents(unsigned int n_components)
{
    n_mixture_components_ = n_components;
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

void ompl::control::CEM::setInitialVariance(std::vector<double> initial_variance)
{
    initial_variance_ = initial_variance;
}
