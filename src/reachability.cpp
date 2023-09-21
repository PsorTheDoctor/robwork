#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

using namespace rw::kinematics;
using namespace rw::math;

std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, const State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);
}

int main (int argc, char** argv)
{
    // Load WorkCell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(
        "../src/scene/Scene.wc.xml"
    );
    
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find relevant frames
    MovableFrame::Ptr cylinderFrame = wc->findFrame<MovableFrame > ("Cylinder");
    if (cylinderFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame Cylinder... check model");
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR5");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5... check model");
    }

    MovableFrame::Ptr baseFrame = wc->findFrame<MovableFrame > ("URReference");
    if (cylinderFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame URReference... check model");
    }

    // create Collision Detector
    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;
    double x, y;

    // For every position of base frame
    for (x = -0.5; x < 0.5; x += 0.1) {
        for (y = -0.5; y < 0.5; y += 0.1) {
            
            // Move robot base to this position
            baseFrame->moveTo(Transform3D<>(Vector3D<>(x, y, 0), RPY<>(0, 0, 0)), state);
            collisionFreeSolutions.clear();
            
            // Reachability analysis for a single robot base location
            for (double rollAngle = 0; rollAngle < 360; rollAngle += 1) {
                cylinderFrame->setTransform(Transform3D<> 
                    (Vector3D<> (cylinderFrame->getTransform(state).P()), RPY<> (rollAngle * Deg2Rad, 0, 0)), state);

                std::vector<Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR5, wc, state);
                for (unsigned int i = 0; i < solutions.size(); i++) {
                    robotUR5->setQ(solutions[i], state);
                    if (!detector.inCollision (state)) {
                        collisionFreeSolutions.push_back(solutions[i]);
                        break;
                    }
                }
            }
            // print the number of colision free solutions
            std::cout << "Position of x=" << x << " y=" << y << " has "
                      << collisionFreeSolutions.size() << " collision-free IK solutions." << std::endl;
        }
    }

    // visualize them
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    rw::loaders::PathLoader::storeTimedStatePath(
        *wc, tStatePath, "../src/playbacks/reachability.rwplay"
    );
    return 0;
}
