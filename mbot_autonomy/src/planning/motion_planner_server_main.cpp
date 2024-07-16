#include <planning/motion_planner_server.hpp>
#include <planning/motion_planner.hpp>
#include <lcm/lcm-cpp.hpp>
#include <thread>

int main(int argc, char** argv){
    MotionPlannerParams planner_params = MotionPlannerParams();
    MotionPlanner planner = MotionPlanner(planner_params);
    lcm::LCM lcmConnection("udpm://239.255.76.67:7667?ttl=0");
    MotionPlannerServer server(lcmConnection, planner);

    std::thread serverThread([&server]() {
        server.run();
    });
    while(true){
        lcmConnection.handle();
    }
    serverThread.join();
    return 0;
}