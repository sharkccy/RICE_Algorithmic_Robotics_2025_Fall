///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Chen-En Lin, Vincent Chang
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        class RTP : public base::Planner
        {
        public:
            RTP(const base::SpaceInformationPtr &si);

            ~RTP() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            
            void setup() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }
            
            void clear() override;

            void getPlannerData(base::PlannerData &data) const override;


        protected:
            
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformation *si)
                  : state(si->allocState())
                {
                }

                ~Motion() = default;

                base::State *state{nullptr};

                Motion *parent{nullptr};
            };
        
            void freeMemory();

            base::StateSamplerPtr sampler_;

            double goalBias_{0.05};

            RNG rng_;

            std::vector<Motion*> tree_;
        };

    }  // namespace geometric
}  // namespace ompl

#endif
