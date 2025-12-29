///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Chen-En Lin, Vincent Chang
//////////////////////////////////////

#include "RG-RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>

// TODO: Implement RGRRT as described
ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RG-RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates,
                                &RGRRT::getIntermediateStates, "0,1");
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if(!nn_){
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    }
    
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_){
        nn_->clear();
    }

    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->reachable_set.size() > 0)
            {
                for (auto &reach_motion : motion->reachable_set)
                {
                    if (reach_motion->state)
                        si_->freeState(reach_motion->state);
                    if (reach_motion->control)
                        siC_->freeControl(reach_motion->control);
                    delete reach_motion;
                }
            }
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        
        motion->parent = nullptr;
        motion->steps = 0;
        generateReachableSet(motion);

        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    // Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* From the nearest node, pick the reachable candidate that gets closest to the sampled state*/
        double baseDist = si_->distance(nmotion->state, rstate);
        Motion *bestCand = nullptr;
        double bestCandDist = baseDist;

        for (Motion *cand : nmotion->reachable_set)
        {
            double d = si_->distance(cand->state, rstate);
            if (d < bestCandDist)
            {
                bestCandDist = d;
                bestCand = cand;
            }
        }

        if (bestCand == nullptr)
            continue;

        // Use bestCand->control to propagate from nmotion->state and create actual tree nodes.
        std::vector<base::State *> pstates;
        unsigned int cd = siC_->propagateWhileValid(nmotion->state, bestCand->control,
                                                    bestCand->steps, pstates, true);

        if (cd < siC_->getMinControlDuration())
        {
            // nothing valid along this control
            for (auto &s : pstates)
                si_->freeState(s);
            continue;
        }

        if (addIntermediateStates_)
        {
            Motion *lastmotion = nmotion;
            bool solved = false;
            size_t p = 0;
            for (; p < pstates.size(); ++p)
            {
                /* create a motion for each intermediate state */
                auto *motion = new Motion(siC_);
                // transfer ownership of the state pointer returned by propagateWhileValid
                si_->copyState(motion->state, pstates[p]);
                // free the propagated state since we copied it into motion->state
                si_->freeState(pstates[p]);
                // copy the control
                motion->control = siC_->allocControl();
                siC_->copyControl(motion->control, bestCand->control);
                motion->steps = 1;
                motion->parent = lastmotion;
                lastmotion = motion;
                nn_->add(motion);
                // generate reachable set for the newly added node
                generateReachableSet(motion);

                double dist = 0.0;
                solved = goal->isSatisfied(motion->state, &dist);
                if (solved)
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
            
            for (; p < pstates.size(); ++p)
                si_->freeState(pstates[p]);
            if (solved)
                break;
        }
        else
        {
            // Only add the final propagated state as a single motion
            Motion *motion = new Motion(siC_);
            // copy endpoint state (last element of pstates)
            si_->copyState(motion->state, pstates.back());
            // free all propagated states
            for (auto &s : pstates)
                si_->freeState(s);
            // copy control
            motion->control = siC_->allocControl();
            siC_->copyControl(motion->control, bestCand->control);
            motion->steps = cd;
            motion->parent = nmotion;
            nn_->add(motion);
            generateReachableSet(motion);

            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            if (solv)
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

        // if (addIntermediateStates_)
        // {
        //     // this code is contributed by Jennifer Barry
        //     std::vector<base::State *> pstates;
        //     cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

        //     if (cd >= siC_->getMinControlDuration())
        //     {
        //         Motion *lastmotion = nmotion;
        //         bool solved = false;
        //         size_t p = 0;
        //         for (; p < pstates.size(); ++p)
        //         {
        //             /* create a motion */
        //             auto *motion = new Motion();
        //             motion->state = pstates[p];
        //             // we need multiple copies of rctrl
        //             motion->control = siC_->allocControl();
        //             siC_->copyControl(motion->control, rctrl);
        //             motion->steps = 1;
        //             motion->parent = lastmotion;
        //             lastmotion = motion;
        //             nn_->add(motion);
        //             double dist = 0.0;
        //             solved = goal->isSatisfied(motion->state, &dist);
        //             if (solved)
        //             {
        //                 approxdif = dist;
        //                 solution = motion;
        //                 break;
        //             }
        //             if (dist < approxdif)
        //             {
        //                 approxdif = dist;
        //                 approxsol = motion;
        //             }
        //         }

        //         // free any states after we hit the goal
        //         while (++p < pstates.size())
        //             si_->freeState(pstates[p]);
        //         if (solved)
        //             break;
        //     }
        //     else
        //         for (auto &pstate : pstates)
        //             si_->freeState(pstate);
        // }
        // else
        // {
        //     if (cd >= siC_->getMinControlDuration())
        //     {
        //         /* create a motion */
        //         auto *motion = new Motion(siC_);
        //         si_->copyState(motion->state, rmotion->state);
        //         siC_->copyControl(motion->control, rctrl);
        //         motion->steps = cd;
        //         motion->parent = nmotion;

        //         nn_->add(motion);
        //         double dist = 0.0;
        //         bool solv = goal->isSatisfied(motion->state, &dist);
        //         if (solv)
        //         {
        //             approxdif = dist;
        //             solution = motion;
        //             break;
        //         }
        //         if (dist < approxdif)
        //         {
        //             approxdif = dist;
        //             approxsol = motion;
        //         }
        //     }
        // }
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
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if(nn_){
        nn_->list(motions);
    }

    double delta = siC_->getPropagationStepSize();

    if(lastGoalMotion_ != nullptr)
    {
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    }

    for (auto m : motions)
    {
        if(m->parent){
            if(data.hasControls()){
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            }
            else{
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
            }
        }
        else{
            data.addStartVertex(base::PlannerDataVertex(m->state));
        }
    }
}

void ompl::control::RGRRT::generateReachableSet(Motion *const m)
{
    auto *space = siC_->getControlSpace()->as<RealVectorControlSpace>();
    const auto low = space->getBounds().low;
    const auto high = space->getBounds().high;

    double interval = (high[0] - low[0]) / 10.0;

    for (int i = 0; i < 11; i++)
    {
        Motion *cand = new Motion(siC_);
        cand->parent = m;

        double *ctrlVals = cand->control->as<RealVectorControlSpace::ControlType>()->values;
        ctrlVals[0] = low[0] + interval * i;
        for (size_t j = 1; j < low.size(); j++)
            ctrlVals[j] = rng_.uniformReal(low[j], high[j]);

        cand->steps = siC_->propagateWhileValid(
            m->state, cand->control, rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration()), cand->state);

        if (cand->steps > 0)
            m->reachable_set.push_back(cand);
        else
        {
            // free internally-allocated state/control before deleting the candidate
            if (cand->state)
                si_->freeState(cand->state);
            if (cand->control)
                siC_->freeControl(cand->control);
            delete cand;
        }
    }
}