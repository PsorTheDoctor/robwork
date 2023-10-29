#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/Path.hpp>

#include <iostream>
#include <string>

using namespace std;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::math;
using namespace rw::kinematics;

#define STEPSIZE 1 * Deg2Rad

struct PathChecker {
    WorkCell::Ptr wc;
    Device::Ptr dev;
    CollisionDetector::Ptr col;

    bool inCollision(Q from, Q to) {
        State state = wc->getDefaultState();
        LinearInterpolator<Q> interp(from, to, (from -to).norm2());
        for (double step = STEPSIZE; step < interp.duration(); step += STEPSIZE) {
            dev->setQ(interp.x(step), state);
            if (col->inCollision(state)) 
                return true;
        }
        return false;
    }

    double pathLength(TimedStatePath path) {
        double d = 0;
        Q last = dev->getQ(path.front().getValue());
        for (TimedState ts: path) {
            Q current = dev->getQ(ts.getValue());
            d += (current - last).norm2();
            last = current;
        }
        return d;
    } 
};

int main (int argc, char** argv) {

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(
        "../src/exercises/scenes/kuka_kr16_workcell/Scene.wc.xml"
    );
    if (wc.isNull ())
        RW_THROW ("WorkCell Not Found");

    TimedStatePath path = PathLoader::loadTimedStatePath(
        wc, "../src/exercises/playbacks/rrt_connect.rwplay"
    );
    if (path.empty ())
        RW_THROW ("Path Not Found");

    Device::Ptr dev = wc->findDevice ("KukaKr16");
    if (dev.isNull ())
        RW_THROW ("Device Not Found");

    wc->setDefaultState(path.front().getValue());
    PathChecker check = {wc, dev,
        rw::core::ownedPtr(new CollisionDetector(
            wc, CollisionStrategy::Factory::makeStrategy("PQP")
        ))
    };
    cout << "Path lenght: " << check.pathLength(path) << endl;

    // Pruning algorithm
    int i = 0;
    while (i < (int) path.size() - 2) {
        // Select points
        Q from = dev->getQ(path[i].getValue());
        Q to = dev->getQ(path[i + 2].getValue());

        // Check fro collisions
        if (check.inCollision(from, to)) {
            i++;
        } else {
            // Erase point if no collisions
            path.erase(path.begin() + i + 1);
            if (i > 0) 
                i--;
        }
        cout << "Progress: " << i << "/" << path.size() - 2 << "\r";
    }
    cout << "\nDone" << endl;
    cout << "Path length: " << check.pathLength(path) << endl;
  
    PathLoader::storeTimedStatePath(
        *wc, path, "../src/exercises/playbacks/rrt_connect_pruned.rwplay"
    );
    return 0;
}
