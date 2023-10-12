#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.
#define ESTEPSIZE 0.05

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}

int main(int argc, char** argv) {

    const string wcFile = "../src/scenes/kuka_kr16_workcell/Scene.wc.xml";
    const string deviceName = "KukaKr16";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
    rw::math::Math::seed();  // Sets the random seed

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);

    //Find the tool and bottle frame
    Frame *tool_frame = wc->findFrame("Tool");
    Frame *bottle_frame = wc->findFrame("Bottle");

    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    //Get the state
    State state =  wc->getDefaultState();

    //These Q's contains the start and end configurations
    Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    Q to(6,1.571, 0.006, 0.030, 0.153, 0.762, 4.490);

    //Set Q to the initial state and grip the bottle frame
    device->setQ(from, state);
    Kinematics::gripFrame(bottle_frame, tool_frame, state);

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state);

    QSampler::Ptr sampler = QSampler::makeConstrained(
        QSampler::makeUniform(device), constraint.getQConstraintPtr()
    );
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(
        constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTConnect
    );

    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    cout << "Planning from " << from << " to " << to << endl;
    QPath path;
    Timer t;
    t.resetAndResume();

    // Query the planner to find a trajectory between the configurations
    planner->query(from, to, path, MAXTIME);
    t.pause();

    if (t.getTime() >= MAXTIME) {
        cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
    }

    // Visualize the movement with an rwplay file
    if (path.size() >=2 ) {
        double distance = 0;
        Q last (path.front());
        for (Q q: path) {
            distance += (q - last).norm2();
        }
        double time = 0.0;
        TimedStatePath tStatePath;
        for (size_t i = 0; i < path.size(); i++) {
            device->setQ(path.at(i), state);
            tStatePath.push_back(TimedState(time, state));
            time += 0.01;
        }
        rw::loaders::PathLoader::storeTimedStatePath(
            *wc, tStatePath, "../src/playbacks/rrt_connect.rwplay"
        );
    }

	return 0;
}
