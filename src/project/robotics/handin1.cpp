#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
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

template<class T>
TimedStatePath recordRobotMovement(
    Device::Ptr robot, const T& interp, State state, string filename
) {
    TimedStatePath res;
    ofstream file;
    file.open("../src/project/data/" + filename);
    string line = "t,q0,q1,q2,q3,q4,q5,x,y,z,roll,pitch,yaw";
    file << line << endl;

    for (double i = 0; i < interp.duration(); i += 0.05) {
        Q q = interp.x(i);
        robot->setQ(q, state);
        res.push_back(TimedState(i, state));

        Vector3D<> pos = robot->baseTend(state).P();
        Rotation3D<> rot = robot->baseTend(state).R();
        RPY<> rpy = RPY<>(rot);

        line = to_string(i) + "," +
            // Joints
            to_string(q[0]) + "," + to_string(q[1]) + "," + to_string(q[2]) + "," + 
            to_string(q[3]) + "," + to_string(q[4]) + "," + to_string(q[5]) + "," + 
            // Position
            to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]) + "," +
            // Orientation
            to_string(rpy[0]) + "," + to_string(rpy[1]) + "," + to_string(rpy[2]);
        file << line << endl;
    }
    file.close();
    return res;
}
 
Trajectory<Q>::Ptr linInterp(
    std::vector<Q> list, double duration, bool doBlend = true, double blendiness = 0.2)
{
    // Create Interpolator
    InterpolatorTrajectory<Q>::Ptr interp = rw::core::ownedPtr (new InterpolatorTrajectory<Q>());

    // Make Sanity Checks
    if (list.size() < 2)
        RW_THROW ("Can't interpolate with 1 point");
    if (list.size() == 2) {
        interp->add(LinearInterpolator<Q>(list[0], list[1], duration));
        return interp;
    }
    if (blendiness > 1)
        blendiness = 1;
    if (blendiness < 1e-5)
        blendiness = 1e-5;

    // Calculate Length of path
    double length = 0;
    for (size_t i = 0; i < list.size() - 1; i++)
        length += (list[i] - list[i + 1]).norm2();

    double durationPerLen = duration / length;

    // Create Linear Interpolators
    std::vector<LinearInterpolator<Q>> linInterps;
    for (size_t i = 1; i < list.size(); i++) {
        linInterps.push_back(LinearInterpolator<Q>(
            list[i - 1], list[i], durationPerLen * (list[i] - list[i - 1]).norm2()));
    }

    // Gather and blend the interpolators
    for (size_t i = 0; i < linInterps.size (); i++) {
        // Can't do blend from pos 0
        if (doBlend && i != 0) {
            // Calculate the maximum blending time
            double maxDuration = linInterps[i].duration() / 2.0;
            if (linInterps[i - 1].duration() > linInterps[i].duration())
                maxDuration = linInterps[i - 1].duration() / 2.0;

            // Apply blend reduction
            maxDuration *= blendiness;
   
            ParabolicBlend<Q> blend(linInterps[i - 1], linInterps[i], maxDuration);
            interp->add(blend, linInterps[i]);
        }
        else {
            interp->add(linInterps[i]);
        }
    }
    return interp;
}

int main (int argc, char** argv)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("../src/project/workcell1/Scene.wc.xml");
    if (wc.isNull())
        RW_THROW("COULD NOT LOAD scene... check path!");

    Device::Ptr robot = wc->findDevice("UR-6-85-5-A");
    if (robot.isNull()) 
        RW_THROW("COULD not find device UR5... check model");

    Device::Ptr gripper = wc->findDevice("WSG50");
    if (gripper.isNull()) 
        RW_THROW("COULD not find device wsg50... check model");

    State state = wc->getDefaultState();
    Q q (0.05);
    gripper->setQ(q, state);  // open gripper

    double duration = 5;
    double blendiness;

    cout << "Enter blendiness value: ";
    cin >> blendiness;

    Q q1 (-2.04046, -1.55823, 2.51484, -0.959757, 1.076, -3.11803); // picking pose
    Q q2 (-2.13653, -2.06284, 1.82038, 0.242461, 0.979933, -3.11646);
    Q q3 (-1.69089, -1.92724, 0.918445, -0.545223, -1.5708, -3.10339);
    Q q4 (-0.317598, -1.50936, -1.86096, -2.97261, -0.330426, -3.06312);
    Q q5 (-0.533163, -1.93222, -2.03989, -2.34841, -0.545764, -3.08773); // placing pose

    // Grasping from side
    Trajectory<Q>::Ptr traj = linInterp({q1, q2, q3, q4, q5}, duration, false);
    TimedStatePath pathQMotion = recordRobotMovement(
        robot, *traj, state, "linear_from_side2.csv"
    );
    PathLoader::storeTimedStatePath(
        *wc, pathQMotion, "../src/project/playbacks/linear_from_side.rwplay"
    );
    Trajectory<Q>::Ptr trajBlend = linInterp({q1, q2, q3, q4, q5}, duration, true, blendiness);
    TimedStatePath pathQBlendMotion = recordRobotMovement(
        robot, *trajBlend, state, "parabolic_from_side2.csv"
    );
    PathLoader::storeTimedStatePath(
        *wc, pathQBlendMotion, "../src/project/playbacks/parabolic_from_side.rwplay"
    );

    Q q1_ (-1.79931, -1.50653, 1.90285, -1.88511, -1.57704, -0.190939);
    Q q2_ (2.11914, -0.876819, -1.80491, -2.09356, 1.62377, 0.584005);
    Q q3_ (-1.69089, -1.92724, 0.918445, -0.545223, -1.5708, -3.10339);
    Q q4_ (-0.252392, -1.31078, -1.40789, -1.98575, 1.48896, -1.78306);
    Q q5_ (-0.817914, -1.81896, -1.7126, -1.13, 1.50615, -2.35304);

    // Grasping from above
    traj = linInterp({q1_, q2_, q3_, q4_, q5_}, duration, false);
    pathQMotion = recordRobotMovement(
        robot, *traj, state, "linear_from_above2.csv"
    );
    PathLoader::storeTimedStatePath(
        *wc, pathQMotion, "../src/project/playbacks/linear_from_above.rwplay"
    );
    trajBlend = linInterp({q1_, q2_, q3_, q4_, q5_}, duration, true, blendiness);
    pathQBlendMotion = recordRobotMovement(
        robot, *trajBlend, state, "parabolic_from_above2.csv"
    );
    PathLoader::storeTimedStatePath(
        *wc, pathQBlendMotion, "../src/project/playbacks/parabolic_from_above.rwplay"
    );
    return 0;
}
