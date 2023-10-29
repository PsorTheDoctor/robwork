#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/Path.hpp>

#include <filesystem>

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;

template<class T>
TimedStatePath recordRobotMovement(Device::Ptr robot, const T& interp, State state) {
    TimedStatePath res;

    for (double i = 0; i < interp.duration(); i += 0.05) {
        robot->setQ(interp.x(i), state);
        res.push_back(TimedState(i, state));
    }
    return res;
}

TimedStatePath linInterp(Device::Ptr robot, State state, Q from, Q to, double duration) {
    LinearInterpolator<Q> interp(from, to, duration);
    return recordRobotMovement(robot, interp, state);
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
            double maxDuration = linInterps[i].duration () / 2.0;
            if (linInterps[i - 1].duration() > linInterps[i].duration())
                maxDuration = linInterps[i - 1].duration () / 2.0;

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
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("../src/project/workcell/Scene.wc.xml");
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

    Q q1 (-2.04046, -1.55823, 2.51484, -0.959757, 1.076, -3.11803); // picking pose
    Q q2 (-2.1292, -1.66916, 2.62991, -0.960751, 0.987263, -3.11646);
    Q q3 (-2.13653, -2.06284, 1.82038, 0.242461, 0.979933, -3.11646);
    Q q4 (-1.69089, -1.92724, 0.918445, -0.545223, 0.658897, -3.10339);
    Q q5 (-0.317598, -1.50936, -1.86096, -2.97261, -0.330426, -3.06312);
    Q q6 (-0.317598, -1.82612, -2.19964, -2.31715, -0.330426, -3.06312);
    Q q7 (-0.533163, -1.93222, -2.03989, -2.34841, -0.545764, -3.08773); // placing pose

    Trajectory<Q>::Ptr traj  = linInterp({q1, q2, q3, q4, q5, q6, q7}, 5, false);
    TimedStatePath pathQMotion = recordRobotMovement(robot, *traj, state);
    PathLoader::storeTimedStatePath(*wc, pathQMotion, "../src/project/playbacks/linear.rwplay");

    Trajectory<Q>::Ptr trajBlend = linInterp({q1, q2, q3, q4, q5, q6, q7}, 5, true, 0.1);
    TimedStatePath pathQBlendMotion = recordRobotMovement(robot, *trajBlend, state);
    PathLoader::storeTimedStatePath(*wc, pathQBlendMotion, "../src/project/playbacks/parabolic.rwplay");
    return 0;
}

