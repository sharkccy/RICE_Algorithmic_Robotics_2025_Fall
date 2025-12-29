///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Chen-En Lin
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <cmath>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <fstream>
#include <iomanip>

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        projection[0] = s[0];
        projection[1] = s[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    const double theta = q[0];
    const double rot_velocity= q[1];

    qdot.resize(q.size(), 0);
    
    qdot[0] = rot_velocity;
    qdot[1] = (-9.81) * cos(theta) + torque;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    auto sSpace = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    
    ompl::base::RealVectorBounds sbounds(2);
    sbounds.setLow(0,-M_PI);
    sbounds.setHigh(0,M_PI);
    sbounds.setLow(1,-10);
    sbounds.setHigh(1,10);
    sSpace->setBounds(sbounds);

    sSpace->registerDefaultProjection(std::make_shared<PendulumProjection>(sSpace.get()));

    auto cSpace = std::make_shared<ompl::control::RealVectorControlSpace>(sSpace, 1);

    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cSpace->setBounds(cbounds);

    auto ss = std::make_shared<ompl::control::SimpleSetup>(cSpace);

    ss->setStateValidityChecker([](const ompl::base::State* /*state*/) {
        return true;
    });

    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);

    auto postPropagate = [](const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/,
                            double /*duration*/, ompl::base::State* result)
    {
        auto* rv = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        while (rv[0] >  M_PI) rv[0] -= 2.0 * M_PI;
        while (rv[0] < -M_PI) rv[0] += 2.0 * M_PI;
    };
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, postPropagate));

    auto si = ss->getSpaceInformation();
    si->setPropagationStepSize(0.02);
    si->setMinMaxControlDuration(1, 20);

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(sSpace);
    start[0] = -M_PI / 2.0;
    start[1] = 0.0;

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(sSpace);
    goal[0] = M_PI / 2.0;
    goal[1] = 0.0;

    ss->setStartAndGoalStates(start, goal, 0.05);

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    ompl::base::PlannerPtr planner;
    auto si = ss->getSpaceInformation();
    
    if (choice == 1) // RRT
    {
        planner = std::make_shared<ompl::control::RRT>(si);
    }
    else if (choice == 2) // KPIECE1
    {
        planner = std::make_shared<ompl::control::KPIECE1>(si);
    }
    else if (choice == 3) // RG-RRT
    {
        planner = std::make_shared<ompl::control::RGRRT>(si);
    }

    ss->setPlanner(planner);

    ompl::base::PlannerStatus solved = ss->solve(30.0);
    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        auto path = ss->getSolutionPath();
        path.printAsMatrix(std::cout);

        std::ofstream out("pendulum_path.csv");
        out << std::fixed << std::setprecision(6);
        out << "t,theta,omega\n";

        const auto &states    = path.getStates();
        const auto &durations = path.getControlDurations();

        double t = 0.0;

        auto *s0 = states.front()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        out << t << "," << s0[0] << "," << s0[1] << "\n";

        for (std::size_t i = 0; i < durations.size(); ++i)
        {
            t += durations[i];
            auto *s = states[i + 1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            out << t << "," << s[0] << "," << s[1] << "\n";
        }

        out.close();
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum
    double runtime_limit = 10.0;
    double memory_limit = 10000.0;
    int run_count = 50;

    ss->setup();

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark benchmark(*ss, "Pendulum Benchmark");

    auto rrt = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    rrt->setName("RRT");
    //RRT->setRange();
    benchmark.addPlanner(rrt);

    auto kpiece = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    kpiece->setName("KPIECE");
    benchmark.addPlanner(kpiece);

    auto rgrrt = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
    rgrrt->setName("RG-RRT");
    //rgrrt->setRange();
    benchmark.addPlanner(rgrrt);

    benchmark.benchmark(request);
    std::cout << "\nBenchmark completed!" << std::endl;
    benchmark.saveResultsToFile("pendulum_benchmark.log");
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
