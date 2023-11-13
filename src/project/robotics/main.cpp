#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/Path.hpp>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::invkin;

vector<Q> getCollisionFreeSolutions(
    SerialDevice::Ptr robot, WorkCell::Ptr wc, Transform3D<> target, State state, bool verbose) 
{
    vector<Q> collisionFreeSolutions;

    CollisionDetector detector(
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()
    );
    ClosedFormIKSolverUR::Ptr solver = rw::core::ownedPtr(
        new ClosedFormIKSolverUR(robot, state)
    );
    // target should be defined as pick or place frame.
    vector<Q> solutions = solver->solve(target, state);

    for (Q q: solutions) {
        robot->setQ(q, state);
        if (!detector.inCollision(state))
            collisionFreeSolutions.push_back(q);
    }
    if (verbose == true) {
        cout << "Solutions: " << solutions.size() << endl;
        cout << "Collsion free solutions: " << collisionFreeSolutions.size() << endl;
    }
    return solutions;  // collisionFreeSolutions;
}

vector<Q> defineTrajectory(
    SerialDevice::Ptr robot, vector<Q> collisionFreeSolutions, State state, double duration
) {
    Q q1, q2, q3, q4, q5;
    if (collisionFreeSolutions.size() > 0) {
        LinearInterpolator<Q> interp(robot->getQ(state), collisionFreeSolutions[0], duration);

        for (double i = 0; i < duration; i += 0.05) {
            robot->setQ(interp.x(i), state);

            if (i >= duration / 5 && q1.size() == 0)
                q1 = interp.x(i);

            if (i >= 2 * duration / 5 && q2.size() == 0)
                q2 = interp.x(i);

            if (i >= 3 * duration / 5 && q3.size() == 0)
                q3 = interp.x(i);

            if (i >= 4 * duration / 5 && q4.size() == 0)
                q4 = interp.x(i);

            if (i >= duration - 0.05 && q5.size() == 0)
                q5 = interp.x(i);
        }
    }
    vector<Q> trajectory = {q1, q2, q3, q4, q5};
    return trajectory;
}

TimedStatePath linearInterp(
    SerialDevice::Ptr robot, vector<Q> trajectory, vector<LinearInterpolator<Q>> interps, State state, double duration
) {
    TimedStatePath res;
    ofstream file;
    file.open("../src/project/data/linear.csv", ios::out | ios::app);
    string line;
    
    for (Q q: trajectory) {
        LinearInterpolator<Q> interp(robot->getQ(state), q, duration);
        interps.push_back(interp);

        for (double i = 0; i < duration; i += 0.05) {
            robot->setQ(interp.x(i), state);
            res.push_back(TimedState(i, state));

            line = to_string(i) + "," + 
                to_string(robot->baseTend(state).P()(0)) + "," +
                to_string(robot->baseTend(state).P()(1)) + "," +
                to_string(robot->baseTend(state).P()(2));
            file << line << endl;
        }
    }
    file.close();
    return res;
}

TimedStatePath parabolicInterp(
    SerialDevice::Ptr robot, vector<LinearInterpolator<Q>> interps, State state, double duration
) {
    TimedStatePath res;
    ofstream file;
    file.open("../src/project/data/parabolic.csv", ios::out | ios::app);
    string line;

    if (interps.size() > 1) {
        for (unsigned int i = 0; i < interps.size() - 1; i++) {
            ParabolicBlend<Q> interp = ParabolicBlend<Q>(interps[i], interps[i + 1], duration / 2);

            for (double i = 0; i < duration; i += 0.05) {
                robot->setQ(interp.x(i), state);
                res.push_back(TimedState(i, state));

                line = to_string(i) + "," + 
                    to_string(robot->baseTend(state).P()(0)) + "," +
                    to_string(robot->baseTend(state).P()(1)) + "," +
                    to_string(robot->baseTend(state).P()(2));
                file << line << endl;
            }
        }
    }
    file.close();
    return res;
}

int main(int argc, char **argv) {

    WorkCell::Ptr wc = WorkCellLoader::Factory::load("../src/project/workcell/Scene.wc.xml");
    if (wc.isNull()) 
        RW_THROW("COULD NOT LOAD scene... check path!");

    SerialDevice::Ptr robot= wc->findDevice<SerialDevice>("UR-6-85-5-A");
    if (robot.isNull())
        RW_THROW("COULD not find device UR-6-85-5-A... check model");

    MovableFrame::Ptr bottle = wc->findFrame<MovableFrame>("Bottle");
    if (bottle.isNull())
        RW_THROW("COULD not find frame Bottle... check model");

    MovableFrame::Ptr bottle2 = wc->findFrame<MovableFrame>("Bottle2");
    if (bottle.isNull())
        RW_THROW("COULD not find frame Bottle2... check model");

    string robotBaseName = robot->getName() + "." + "Base";
    Frame::Ptr robotBase = wc->findFrame(robotBaseName);
    if (robotBase.isNull())
        RW_THROW("COULD not find frame robotBase... check model");

    string robotTcpName = robot->getName() + "." + "TCP";
    Frame::Ptr robotTcp = wc->findFrame(robotTcpName);
    if (robotTcp.isNull())
        RW_THROW("COULD not find frame robotTcp... check model");

    Frame::Ptr gripper = wc->findFrame("GraspTCP");
    if (gripper.isNull())
        RW_THROW("COULD not find frame GraspTCP... check model");

    Frame::Ptr graspFromSide = wc->findFrame("GraspFromSide");
    if (gripper.isNull())
        RW_THROW("COULD not find frame GraspTCP... check model");

    Frame::Ptr graspFromAbove = wc->findFrame("GraspFromAbove");
    if (gripper.isNull())
        RW_THROW("COULD not find frame GraspTCP... check model");
    
    State state = wc->getDefaultState();
    double duration = 5;
    string playbackPath = "../src/project/playbacks/";

    // Attach gripper to the end effector
    Transform3D<> gripperTrobotTcp = Kinematics::frameTframe(gripper, robotTcp, state);

    Transform3D<> baseTpick;
    Transform3D<> baseTplace;
    Transform3D<> pickT;
    Transform3D<> placeT;

    Vector3D<> initialBottlePos = bottle->getTransform(state).P();
    Vector3D<> desiredBottlePos(0.3, -0.47, 0.21);

    vector<Q> collisionFreeSolutions;
    vector<Q> trajectory;
    vector<LinearInterpolator<Q>> interps;
    TimedStatePath t;

    /*
    Pick and place from side
    */
    // Move the bottle to desired position, capture the transform, and bring it back
    Frame::Ptr pickFromSide = graspFromSide;
    baseTpick = Kinematics::frameTframe(robotBase, pickFromSide, state);
    bottle->moveTo(Transform3D<>(desiredBottlePos, RPY<>(-90 * Deg2Rad, 0, 90 * Deg2Rad)), state);

    Frame::Ptr placeFromSide = graspFromSide;
    baseTplace = Kinematics::frameTframe(robotBase, placeFromSide, state);
    bottle->moveTo(Transform3D<>(initialBottlePos, RPY<>(-90 * Deg2Rad, 0, 90 * Deg2Rad)), state);
    
    // Pick from side
    pickT = baseTpick * gripperTrobotTcp;
    collisionFreeSolutions = getCollisionFreeSolutions(robot, wc, pickT, state, true);
    trajectory = defineTrajectory(robot, collisionFreeSolutions, state, duration);
    t = linearInterp(robot, trajectory, interps, state, duration);
    PathLoader::storeTimedStatePath(*wc, t, playbackPath + "linear/pick_from_side.rwplay");
    // t = parabolicInterp(robot, interps, state, duration);
    // PathLoader::storeTimedStatePath(*wc, t, playbackPath + "parabolic/pick_from_side.rwplay");

    // Place from side
    placeT = baseTplace * gripperTrobotTcp;
    collisionFreeSolutions = getCollisionFreeSolutions(robot, wc, placeT, state, true);
    trajectory = defineTrajectory(robot, collisionFreeSolutions, state, duration);
    t = linearInterp(robot, trajectory, interps, state, duration);
    PathLoader::storeTimedStatePath(*wc, t, playbackPath + "linear/place_from_side.rwplay");
    // t = parabolicInterp(robot, interps, state, duration);
    // PathLoader::storeTimedStatePath(*wc, t, playbackPath + "parabolic/place_from_side.rwplay");

    /*
    Pick and place from above
    */
    // Move the bottle to desired position, capture the transform, and bring it back
    baseTpick = Kinematics::frameTframe(robotBase, graspFromAbove, state);
    bottle->moveTo(Transform3D<>(desiredBottlePos, RPY<>(-90 * Deg2Rad, 0, 90 * Deg2Rad)), state);
    baseTplace = Kinematics::frameTframe(robotBase, graspFromAbove, state);
    bottle->moveTo(Transform3D<>(initialBottlePos, RPY<>(-90 * Deg2Rad, 0, 90 * Deg2Rad)), state);

    // Pick from above
    pickT = baseTpick * gripperTrobotTcp;
    collisionFreeSolutions = getCollisionFreeSolutions(robot, wc, pickT, state, true);
    trajectory = defineTrajectory(robot, collisionFreeSolutions, state, duration);
    t = linearInterp(robot, trajectory, interps,state, duration);
    PathLoader::storeTimedStatePath(*wc, t, playbackPath + "linear/pick_from_above.rwplay");
    // t = parabolicInterp(robot, interps, state, duration);
    // PathLoader::storeTimedStatePath(*wc, t, playbackPath + "parabolic/pick_from_above.rwplay");

    // Place from above
    placeT = baseTplace * gripperTrobotTcp;
    collisionFreeSolutions = getCollisionFreeSolutions(robot, wc, placeT, state, true);
    trajectory = defineTrajectory(robot, collisionFreeSolutions, state, duration);
    t = linearInterp(robot, trajectory, interps,state, duration);
    PathLoader::storeTimedStatePath(*wc, t, playbackPath + "linear/place_from_above.rwplay");
    // t = parabolicInterp(robot, interps, state, duration);
    // PathLoader::storeTimedStatePath(*wc, t, playbackPath + "parabolic/place_from_above.rwplay");

    return 0;
}
