#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/Path.hpp>

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::kinematics;

template< class T >
TimedStatePath recordRobotMovement (Device::Ptr device, const T& interp, State state)
{
    TimedStatePath res;
    for (double i = 0; i < interp.duration(); i += 0.05) {
        device->setQ (interp.x (i), state);
        res.push_back (TimedState (i, state));
    }
    return res;
}

TimedStatePath linearInterp (Device::Ptr device, State state, Q from, Q to, double duration)
{
    LinearInterpolator<Q> interp(from, to, duration);
    return recordRobotMovement(device, interp, state);
}

int main (int argc, char** argv)
{
    // load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load ("../src/exercises/scenes/ur5_workcell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find Device
    Device::Ptr robotUR5 = wc->findDevice ("UR5");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    // define movement
    Q q1 (0.0775973, 5.37959, -2.53825, 0.300232, -4.78999, 0);
    Q q2 (1.53439, 4.78245, -2.19395, 0.553112, -6.24678, 0);

    // Ex. 4.1
    TimedStatePath linearQMotion = linearInterp(robotUR5, wc->getDefaultState(), q1, q2, 5);
    PathLoader::storeTimedStatePath(*wc, linearQMotion, "../src/playbacks/linear.rwplay");
    return 0;
}
