///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Vincent Chang
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        // Use a grid with x, y, 2 dim
        return 2;
    }

    void project(const ompl::base::State *  state , Eigen::Ref<Eigen::VectorXd>  projection ) const override
    {
        // TODO: Your projection for the car
        const ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
        const ompl::base::SE2StateSpace::StateType *se2state = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        projection[0] = se2state->getX();
        projection[1] = se2state->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control *  control ,
            ompl::control::ODESolver::StateType &  qdot )
{
    // TODO: Fill in the ODE for the car's dynamics
    // state q: [x, y, theta, v]
    // const double x = q[0];
    // const double y = q[1];
    const double theta = q[2];
    const double v = q[3];

    // control u: [omega, v_dot]
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double omega = u[0];
    const double v_dot = u[1];

    // qdot: [x_dot, y_dot, theta_dot, v_dot] = [v*cos(theta), v*sin(theta), omega, v_dot]
    qdot.resize(4);
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = omega;
    qdot[3] = v_dot;
}

void makeStreet(std::vector<Rectangle> &  obstacles )
{
    // TODO: Fill in the vector of rectangles with your street environment.
    obstacles.clear();
    obstacles.push_back({0.0, 20.0, 40.0, 80.0});
    obstacles.push_back({60.0, 20.0, 40.0, 60.0});

    std::ofstream obsout("obstacles.csv");
    for (const auto &o : obstacles){
        obsout << o.x << ',' << o.y << ',' << o.width << ',' << o.height << '\n';
    }
    obsout.close();
    std::cout << "Saved: obstacles.csv" << std::endl;
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles )
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto se2StateSpace = std::make_shared<ompl::base::SE2StateSpace>();
    auto v1StateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(1);

    ompl::base::RealVectorBounds xyBounds(2);
    xyBounds.setLow(0, 0.0);
    xyBounds.setHigh(0, 100.0);
    xyBounds.setLow(1, 0.0);
    xyBounds.setHigh(1, 100.0);
    se2StateSpace->setBounds(xyBounds);

    ompl::base::RealVectorBounds vBounds(1);
    vBounds.setLow(0, -10.0);   // min speed
    vBounds.setHigh(0, 10.0); // max speed
    v1StateSpace->setBounds(vBounds);

    auto StateSpace = std::make_shared<ompl::base::CompoundStateSpace>();
    StateSpace->addSubspace(se2StateSpace, 1.0);
    StateSpace->addSubspace(v1StateSpace, 1.0);
    
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(StateSpace, 2);
    ompl::base::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1.0);   // min omega
    controlBounds.setHigh(0, 1.0); // max omega
    controlBounds.setLow(1, -2.0);   // min acceleration
    controlBounds.setHigh(1, 2.0); // max acceleration
    controlSpace->setBounds(controlBounds);

    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    auto si = ss->getSpaceInformation();
    si->setPropagationStepSize(0.02);
    si->setMinMaxControlDuration(1, 20);
    si->setStateValidityCheckingResolution(0.003);

    // Set state validity checker
    const double carLength = 3;
    ss->setStateValidityChecker([si, carLength, vBounds, &obstacles](const ompl::base::State *s) -> bool {

        if(!si->satisfiesBounds(s))
            return false;

        const auto *cstate = s->as<ompl::base::CompoundState>();
        const auto *se2state = cstate->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto *vstate = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);

        const double v = vstate->values[0];
        if (v < vBounds.low[0] || v > vBounds.high[0]){
            return false;
        }

        return isValidStateSquare(se2state, carLength, obstacles);

    });
    
    StateSpace->registerDefaultProjection(std::make_shared<CarProjection>(StateSpace.get()));

    ompl::base::ScopedState<> start(StateSpace), goal(StateSpace);
    start->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(10.0, 10.0);
    start->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    start->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;

    goal->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(90.0, 90.0);
    goal->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    goal->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;

    ss->setStartAndGoalStates(start, goal, 5.0);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    if (choice == 1)
    {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    }
    else if (choice == 2)
    {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    }
    else if (choice == 3)
    {
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    }

    ss->setup();
    auto solved = ss->solve(10.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        std::cout << "Number of states in solution: " << ss->getSolutionPath().asGeometric().getStateCount() << std::endl;

        std::ofstream bout("path_box.csv");
        auto gpath = ss->getSolutionPath().asGeometric();
        // gpath.interpolate(400);

        for (std::size_t i = 0; i < gpath.getStateCount(); ++i)
        {
            const ompl::base::State *s = gpath.getState(i);

            // state is compound: [SE2, R^1]
            const auto *cs  = s->as<ompl::base::CompoundState>();
            const auto *se2 = cs->as<ompl::base::SE2StateSpace::StateType>(0);
            const auto *sv  = cs->as<ompl::base::RealVectorStateSpace::StateType>(1);

            const double x   = se2->getX();
            const double y   = se2->getY();
            const double yaw = se2->getYaw();
            const double v   = (*sv)[0];

            bout << x << ',' << y << ',' << yaw << ',' << v << '\n';
        }
        bout.close();
        std::cout << "Saved: path_box.csv" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the car
    double runtime_limit = 10.0;
    double memory_limit = 10000.0;
    int run_count = 50;

    ss->setup();

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark benchmark(*ss, "Car Benchmark");

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
    benchmark.saveResultsToFile("car_benchmark.log");
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
