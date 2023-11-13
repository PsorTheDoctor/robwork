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

    const string wcFile = "../src/project/workcell2/Scene.wc.xml";
    const string deviceName = "UR-6-85-5-A";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
    rw::math::Math::seed();  // Sets the random seed

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);

    //Find the tool and bottle frame
    Frame *tool_frame = wc->findFrame("GraspTCP");
    Frame *bottle_frame = wc->findFrame("Bottle");

    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    // Pick configurations
    Q q1 (1.748, -2.575, -1.407, 0.844, -1.752, 0.001);
    Q q2 (1.80076, -1.55807, 1.99107, -2.00381, -1.5708, -0.229965);
    Q q3 (2.28404, -1.25367, 1.94768, -2.26481, -1.5708, -0.713246);
    Q q4 (2.28216, 0.59116, -1.992, -0.16996, -1.5708, -0.711361);
    Q q5 (1.26711, 0.589782, -1.98343, -0.177168, -1.5708, 0.303687);
    Q q6 (1.29277, -1.2514, 1.97758, -2.29567, -1.57165, 0.278031);

    vector<Q> pickConfigs = {q2, q3, q4, q5, q6};

    // Place configuration
    Q to (0.895895, -2.0774, -1.57074, -1.13745, 1.56919, 0.652003);

    vector<double> stepSizes = {0.01, 0,05, 0.1};
    
    for (Q from: pickConfigs) {
        for (double eps: stepSizes) {
            State state =  wc->getDefaultState();

            // Set Q to the initial state and grip the bottle frame
            device->setQ(from, state);
            Kinematics::gripFrame(bottle_frame, tool_frame, state);

            CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
            PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state);

            QSampler::Ptr sampler = QSampler::makeConstrained(
                QSampler::makeUniform(device), constraint.getQConstraintPtr()
            );
            QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(
                constraint, sampler, metric, eps, RRTPlanner::RRTConnect
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

            if (t.getTime() >= MAXTIME)
                cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;

            // Visualize the movement with an rwplay file
            if (path.size() >= 2) {
                double distance = 0;
                Q last(path.front());
                for (Q q: path) 
                    distance += (q - last).norm2();

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
        }
    }
	return 0;
}
