///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Chen-En Lin, Vincent Chang
//////////////////////////////////////

#include "RTP.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathGeometric.h"

// TODO: Implement RTP as described
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si) : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;

    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::setup()
{
    base::Planner::setup();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
}

void ompl::geometric::RTP::freeMemory()
{
    for (auto &motion : tree_)
    {
        if (motion->state)
            si_->freeState(motion->state);
        delete motion;
    }
    tree_.clear();
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    // basic checks and goal handling
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // 2. add start states to tree_
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_.get());
        si_->copyState(motion->state, st);
        tree_.push_back(motion);
    }

    // verify we have at least one start
    if (tree_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // ensure sampler exists
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size());

    // solution and approximate solution pointers
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    // allocate a random state to sample
    auto *rmotion = new Motion(si_.get());
    base::State *rstate = rmotion->state;

    // ptc is the termination condition (e.g., time limit, goal found, etc.)
    while (!ptc())
    {
        // sample a random state (with small probability sample the goal)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
        }
        else
        {
            sampler_->sampleUniform(rstate);
        }

        //pick a random state from the tree
        std::size_t idx = (std::size_t)rng_.uniformInt(0, static_cast<int>(tree_.size()) - 1);
        Motion *nmotion = tree_[idx];

        base::State *dstate = rstate;

        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion and attached to the random motion we picked in the tree
            auto *motion = new Motion(si_.get());
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            tree_.push_back(motion);

            // check if we have reached the goal
            double dist = 0.0;
            bool sat = pdef_->getGoal()->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *m = solution;
        while (m != nullptr)
        {
            mpath.push_back(m);
            m = m->parent;
        }

        // reverse the path
        auto path = std::make_shared<PathGeometric>(si_);
        for (int i = static_cast<int>(mpath.size()) - 1; i >= 0; --i)
        {
            path->append(mpath[i]->state);
        }   

        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    // clean up
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), tree_.size());
    return {solved, approximate};
}

void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion *> motions = tree_;
    
    for (auto m : motions)
    {
        if (m->parent)
        {
            data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
        {
            data.addStartVertex(base::PlannerDataVertex(m->state));
        }
    }
}