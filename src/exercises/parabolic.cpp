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
    for (double i = 0; i < interp.duration (); i += 0.05) {
        device->setQ (interp.x (i), state);
        res.push_back (TimedState (i, state));
    }
    return res;
}

Trajectory<Q>::Ptr parabolicInterp(std::vector<Q> list, double duration, double blendiness = 0.2)
{
    InterpolatorTrajectory<Q>::Ptr interp = rw::core::ownedPtr(
        new InterpolatorTrajectory<Q>()
    );
    if (list.size() < 2) 
        RW_THROW("Can't interpolate with 1 point.");
    if (list.size() == 2) {
        interp->add(LinearInterpolator<Q> (list[0], list[1], duration));
        return interp;
    }
    if (blendiness > 1) {
        blendiness = 1;
    } else if (blendiness < 1e-5) {
        blendiness = 1e-5;
    }

    // Calculate length of the path
    double length = 0;
    for (size_t i = 0; i < list.size() - 1; i++) {
        length += (list[i] - list[i + 1]).norm2();
    }
    double durationPerLength = duration / length;

    // Create linear interpolators
    std::vector<LinearInterpolator<Q>> linearInterps;
    for (size_t i = 1; i < list.size(); i++) {
        linearInterps.push_back(LinearInterpolator<Q>(
            list[i - 1], list[1], durationPerLength * (list[i] - list[i - 1]).norm2()
        ));
    }
    // Gather and blend the interpolators
    for (size_t i = 0; i < linearInterps.size(); i++) {
        // Cant't do blend from pos 0
        if (i != 0) {
            // Calculate the maximum blending time
            double maxDuration = linearInterps[i].duration() / 2.0;
            if (linearInterps[i - 1].duration() > linearInterps[i].duration())
                maxDuration = linearInterps[i - 1].duration() / 2.0;

            maxDuration *= blendiness;  // Apply blend reduction
            std::cout << (linearInterps[i - 1].getEnd() - linearInterps[i].getStart()) << std::endl;
            ParabolicBlend<Q> blend (linearInterps[i - 1], linearInterps[i], maxDuration);
            interp->add(blend, linearInterps[i]);  
        } else {
            interp->add(linearInterps[i]);
        }
    }
    return interp;
}

int main (int argc, char** argv)
{
    // load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load ("../src/exercises/scenes/ur5_workcell/Scene.wc.xml");
    if (wc.isNull ())
        RW_THROW ("COULD NOT LOAD scene... check path!");

    // find Device
    Device::Ptr robotUR5 = wc->findDevice ("UR5");
    if (robotUR5.isNull ()) 
        RW_THROW ("COULD not find device UR5 ... check model");

    // define movement
    Q q1 (0.0775973, 5.37959, -2.53825, 0.300232, -4.78999, 0);
    Q q2 (1.53439, 4.78245, -2.19395, 0.553112, -6.24678, 0);
    Q q3 (1.53439, 4.8908, -1.49275, -0.256476, -6.24678, 0);
    Q q4 (0.0775973, 5.31938, -1.73337, -0.444431, -4.78999, 0);

    // Ex. 4.2
    Trajectory<Q>::Ptr blend = parabolicInterp({q1, q2, q3, q4}, 5, 0.1);
    TimedStatePath pathQBlendMotion = recordRobotMovement(
        robotUR5, *blend, wc->getDefaultState()
    );
    PathLoader::storeTimedStatePath(*wc, pathQBlendMotion, "../src/playbacks/parabolic.rwplay");
    return 0;
}
