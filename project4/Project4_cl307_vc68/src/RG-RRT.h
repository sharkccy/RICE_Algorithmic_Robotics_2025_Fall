///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Chen-En Lin, Vincent Chang
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

 #include "ompl/control/planners/PlannerIncludes.h"
 #include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace control
    {
        class RGRRT : public base::Planner
        {
            public:
                RGRRT(const SpaceInformationPtr &si);

                ~RGRRT() override;

                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                void clear() override;

                void setGoalBias(double goalBias)
                {
                    goalBias_ = goalBias;
                }

                double getGoalBias() const
                {
                    return goalBias_;
                }

                bool getIntermediateStates() const
                {
                    return addIntermediateStates_;
                }

                void setIntermediateStates(bool addIntermediateStates)
                {
                    addIntermediateStates_ = addIntermediateStates;
                }

                void getPlannerData(base::PlannerData &data) const override;

                template <template <typename T> class NN> // Template template parameter, T is the type parameter of NN

                void setNearestNeighbors()
                {
                    if (nn_ && nn_->size() != 0){
                        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                    }
                    clear();
                    nn_ = std::make_shared<NN<Motion*>>();
                    setup();
                }

                void setup() override;
            
            protected:

                class Motion
                {
                public:
                    Motion() = default;

                    Motion(const SpaceInformation *si)
                        : state(si->allocState()), control(si->allocControl())
                    { 
                    }

                    ~Motion() = default;

                    base::State *state{nullptr};

                    Control *control{nullptr};

                    unsigned int steps{0};

                    Motion *parent{nullptr};

                    std::vector<Motion*> reachable_set;
                };

                void freeMemory();

                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                base::StateSamplerPtr sampler_;

                DirectedControlSamplerPtr controlSampler_;

                const SpaceInformation *siC_;

                std::shared_ptr<NearestNeighbors<Motion *>> nn_;

                double goalBias_{0.05};
                
                bool addIntermediateStates_{false};

                RNG rng_;

                Motion* lastGoalMotion_{nullptr};

                void generateReachableSet(Motion *m);

        };

    }  // namespace control 
}  // namespace ompl

#endif
