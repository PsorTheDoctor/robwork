#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Constants.hpp> // Pi, Deg2Rad
#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

Transform3D<> forwardKinematics(const vector<Transform3D<>>& trefs,
                                const unsigned int idx, const Q& q) 
{
    if (trefs.size() != q.size())
        RW_THROW("The number of local transformations must be equal to the length of the "
                 "configuration vector.");

    Transform3D<> baseTi;

    for (unsigned int i = 0; i < idx; ++i) {
        Transform3D<> Tz (RPY<> (q[i], 0, 0).toRotation3D());
        baseTi = baseTi * trefs[i] * Tz;
    }
    return baseTi;
}

vector<Transform3D<>> generateTrefs() 
{
    // Joint 1
    Vector3D<> V1(0, 0, 0);
    RPY<> R1(0, 0, 0);
    Transform3D<> T1(V1, R1.toRotation3D());

    // Joint 2
    Vector3D<> V2(0, 0, 0.0892);
    RPY<> R2(90 * Deg2Rad, 0, 90 * Deg2Rad);
    Transform3D<> T2(V2, R2.toRotation3D());

    // Joint 3
    Vector3D<> V3(0, 0.425, 0);
    RPY<> R3(270 * Deg2Rad, 0, 0);
    Transform3D<> T3(V3, R3.toRotation3D());

    // Joint 4
    Vector3D<> V4(-0.39243, 0, 0);
    RPY<> R4(0, 0, 0);
    Transform3D<> T4(V4, R4.toRotation3D());

    // Joint 5
    Vector3D<> V5(0, 0, 0.109);
    RPY<> R5(270 * Deg2Rad, 0, 90 * Deg2Rad);
    Transform3D<> T5(V5, R5.toRotation3D());

    // Joint 6
    Vector3D<> V6(0, 0, 0.093);
    RPY<> R6(0, 0, 270 * Deg2Rad);
    Transform3D<> T6(V6, R6.toRotation3D());

    return {T1, T2, T3, T4, T5, T6};
}

int main (int argc, char** argv)
{
    // Robot Configuration
    const Q q(6, 0.859, 0.208, -0.825, -0.746, -1.632, 1.527);

    const vector<Transform3D<>> trefs = generateTrefs();

    const Vector3D<> VTCP(0, 0, 0.082);
    const RPY<> RTCP(270 * Deg2Rad, 0, 0);
    const Transform3D<> TTCP(VTCP, RTCP.toRotation3D());
    const Transform3D<> baseTtcp = forwardKinematics(trefs, 6, q) * TTCP;

    cout << "Manual: \n" << baseTtcp.P() << " " << RPY<>(baseTtcp.R()) << endl;

    // Load WorkCell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(
        "../src/exercises/scenes/ur5_workcell2/Scene.wc.xml"
    ); 
    if (wc.isNull ()) 
        RW_THROW ("WorkCell not Found");

    const string deviceName = "UR-6-85-5-A";

    // Find UR Robot in scene
    Device::Ptr device = wc->findDevice(deviceName);
    if (device.isNull ()) 
        RW_THROW ("Device UR-6-85-5-A was not found!");

    // Get Default Configuration
    State state = wc->getDefaultState ();
    device->setQ (q, state);

    // Compare RobWork Forward Kinematics with your solution
    Frame* tcpFrame = wc->findFrame(deviceName + ".TCP");
    if (tcpFrame == nullptr)
        RW_THROW ("TCP frame not found!");

    const Transform3D<> baseTtool = device->baseTframe(tcpFrame, state);

    cout << "RW:\n" << baseTtool.P() << " " << RPY<>(baseTtool.R()) << endl;

    return 0;
}
