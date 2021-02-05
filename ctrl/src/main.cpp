#include <iostream>
#include "ctrl/sdir_ctrl.h"
#include "ctrl/com/json_handler.h"
#include <string>
#include <stdio.h>
#include <chrono>
#include <thread>

#include <cmath>
#include <iomanip>
#include "ctrl/kinematics/direct/fw_kinematics.h"
#include "ctrl/kinematics/inverse/inverse_kinematics.h"


extern "C" {
#include "ctrl/com/remoteApi/extApi.h"
}

/** \brief main entry point of the sdir project
 *  This is the main class of the sdir programming project. It represents the interface to the extern context. In particular
 *  to the VREP environment. It reads the json string form the gui and interprets the respective operations.
 *  The four main operations are:
 *   - POS_2_CFG: In this mode the receiving json string contains a {@ref SixDPos}. From this, an inverse kinematic must
 *                be computed and the computed {@ref Configuration} is returned.
 *
 *   - CFG_2_POS In this mode the receiving json string contains a {@ref Configuration}. From this, the direct kinematic
 *               must be computed and the computed {@ref SixDPos} is returned.
 *
 *   - PTP In this mode the json string contains a start and and a target configuration. From this a {@ref PTP} movement
 *         is computed and the resulting {@ref Trajectory} is used to move the robot manipulator. This is done by calling
 *         the VREP remote api function runConfig. By using the sleep command the update interval between the configurations
 *         is handled.
 *
 *   - LIN In this mode the json string contains a start and and a target configuration. From this a {@ref LIN} movement
 *         is computed and the resulting {@ref Trajectory} is used to move the robot manipulator. This is done by calling
 *         the VREP remote api function runConfig. By using the sleep command the update interval between the configurations
 *         is handled.
 *
 * As long as you do not change the communication between GUI and controller this class must not be changed.
 * The only thing you should take care of is the update rate for sending configuration values to the robot. Thsi should
 * be synchronized with VREP and your trajectory time resolution.
 */

using namespace std;

simxInt *jh = new simxInt[6];

simxInt Initial() {
    string Joint;
    simxInt *check = new simxInt[1];
    int portNb = 19997;
    simxInt clientID = -1;
    simxFinish(-1);

    clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);

    if (clientID > -1) {
        //Starten der Simulation
        simxStartSimulation(clientID, simx_opmode_oneshot);
        cout << "Simulation started" << endl;

        for (int i = 0; i < 6; i++) {
            //Abfrage der einzelnen Joint Handles des Manipulators
            Joint = "KR120_2700_2_joint" + to_string(i + 1);
            const char *c = Joint.c_str();
            simxGetObjectHandle(clientID, (const simxChar *) c, check, simx_opmode_oneshot_wait);
            jh[i] = check[0];
        }
    } else {
        printf("Connection Fail");
        getchar();
        extApi_sleepMs(10);
    }

    return clientID;
}

vector<SixDPos*> spline_path;

int main() {
    // Example for forward kinematics
    SixDPos *fwkinSample = (new FwKinematics())->get_fw_kinematics(new Configuration(
            {0, -0.5 * M_PI, 0.5 * M_PI, 0, 0, 0}
    ));
    cout << "Forward Kinematics sample: " << endl
         << fixed << setprecision(2)
         << "X=" << fwkinSample->get_X() << " Y=" << fwkinSample->get_Y() << " Z=" << fwkinSample->get_Z()
         << " A=" << fwkinSample->get_A() * 180 / M_PI
         << " B=" << fwkinSample->get_B() * 180 / M_PI
         << " C=" << fwkinSample->get_C() * 180 / M_PI << endl << endl;

    // Example for inverse kinematics
    InvKinematics invKinematics;
    FwKinematics fwKinematics;
    vector<TMatrix*> transformationMatrices;
    transformationMatrices.push_back(new TMatrix(
            -5.07791870e-01, 6.35025673e-01, -5.82142433e-01, -7.18376830e+02 / 1000,
            2.62242221e-01, 7.57620537e-01, 5.97695691e-01, 1.89742719e+03 / 1000,
            8.20595171e-01, 1.50842688e-01, -5.51244092e-01, 2.32490439e+02 / 1000,
            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00
            ));

            transformationMatrices.push_back(new TMatrix(
                    -4.42438962e-02 ,  9.57043695e-01 ,  2.86548154e-01 ,  9.24075420e+01/1000,
     -9.74010246e-01  , 2.24551327e-02 , -2.25388126e-01 , -6.68540585e+02/1000,
     -2.22140762e-01 , -2.89072887e-01 ,  9.31176862e-01 ,  2.74134524e+03/1000,
     0.00000000e+00 ,  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00
    ));

    for (int i = 0; i < transformationMatrices.size(); i++) {
        TMatrix *transformationMatrix = transformationMatrices[i];
        double x = transformationMatrix->get(0, 3);
        double y = transformationMatrix->get(1, 3);
        double z = transformationMatrix->get(2, 3);
        array<double, 3> ea = fwKinematics.calculate_euler_angles(transformationMatrix);
        double roll = ea[0];
        double pitch = ea[1];
        double yaw = ea[2];
        auto configurations = invKinematics.get_inv_kinematics(new SixDPos(x, y, z, yaw, pitch, roll));
        cout << "Inverse Kinematics sample: " << endl;
        transformationMatrix->print();
        cout << fixed << setprecision(2)
             << "X=" << x << " Y=" << y << " Z=" << z
             << " A=" << roll << " B=" << pitch << " C=" << yaw << endl;
        cout << configurations->size() << " Solutions:" << endl;
        for (int j = 0; j < configurations->size(); j++) {
            cout << fixed << setprecision(2)
                 << "R1=" << (*(*configurations)[j])[0]
                    << " R2=" << (*(*configurations)[j])[1]
                    << " R3=" << (*(*configurations)[j])[2]
                    << " R4=" << (*(*configurations)[j])[3]
                    << " R5=" << (*(*configurations)[j])[4]
                    << " R6=" << (*(*configurations)[j])[5]
                    << endl;
        }
        cout << endl;
    }

    auto configurations = invKinematics.get_inv_kinematics(new SixDPos(2.2150217743250376, 0.44999663348281566, 2.359973741582353, -1.8268180825737412, 2.0207665616890536, -1.8268161754190866));
    cout << "Inverse Kinematics sample that breaks: " << endl;
    cout << configurations->size() << " Solutions:" << endl;
    for (int j = 0; j < configurations->size(); j++) {
        cout << fixed << setprecision(2)
             << "R1=" << (*(*configurations)[j])[0]
             << " R2=" << (*(*configurations)[j])[1]
             << " R3=" << (*(*configurations)[j])[2]
             << " R4=" << (*(*configurations)[j])[3]
             << " R5=" << (*(*configurations)[j])[4]
             << " R6=" << (*(*configurations)[j])[5]
             << endl;
    }
    cout << endl;

    cout << "This is the entry point of the SDIR programming project" << endl;
    SdirCtrl ctrl;
    float c[6];
    simxInt ID = Initial();
    simxUChar *value = new simxUChar;
    simxInt length = -1;

    setvbuf(stdout, nullptr, _IONBF, 0);

    simxGetStringSignal(ID, "callsignal", &value, &length, simx_opmode_streaming);

    /*
     * While VREP remote connection is available
     */
    while (simxGetConnectionId(ID) != -1) {
        // read data from vrep
        simxGetStringSignal(ID, "callsignal", &value, &length, simx_opmode_blocking);

        // if data via signal "callsignal" has been received
        if (length > 0) {
            // cast vrep data to string
            string t = string(reinterpret_cast<const char *>(value), length);
//            cout << t << endl;
            // deserialize the json input
            JsonHandler jsonHandler(t);

            /*
             * compute a configuration from a position
             */
            if (jsonHandler.get_op_mode() == OpMode::POS_2_CFG) {
                SixDPos pos(jsonHandler.get_data()[0]);
                Configuration cfg0(jsonHandler.get_data()[1]);
//                cout << jsonHandler.get_json_string(&pos) << endl;
                vector<Configuration *> *result_cfg = new vector<Configuration *>({ctrl.get_next_config_from_pos(&cfg0, &pos)});
                string json_return_string = jsonHandler.get_json_string(result_cfg);
//                cout << json_return_string << endl;
                simxSetStringSignal(ID, "returnsignal",
                                    reinterpret_cast<const simxUChar *>(json_return_string.c_str()),
                                    json_return_string.length(), simx_opmode_oneshot);
            }

            /*
             * compute a position from a configuration
             */
            if (jsonHandler.get_op_mode() == OpMode::CFG_2_POS) {
                Configuration cfg(jsonHandler.get_data()[0]);
//                cout << jsonHandler.get_json_string(&cfg) << endl;
                SixDPos *return_pos = ctrl.get_pos_from_config(&cfg);
                string json_return_string = jsonHandler.get_json_string(return_pos);
//                cout << json_return_string << endl;
                simxSetStringSignal(ID, "returnsignal",
                                    reinterpret_cast<const simxUChar *>(json_return_string.c_str()),
                                    json_return_string.length(), simx_opmode_oneshot);
            }

            /*
             * Compute a PTP trajectory and move the robot asynchronous in VREP
             */
            if (jsonHandler.get_op_mode() == OpMode::PTP) {
                Configuration start_cfg((jsonHandler.get_data())[0]);
                Configuration end_cfg((jsonHandler.get_data())[1]);
//                cout << "Path Configuration" << endl;
//                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
//                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

                Trajectory *trajectory = ctrl.move_robot_ptp(&start_cfg, &end_cfg, false);
                for (Configuration *cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c,
                                           0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                           simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            /*
             * Compute a PTP trajectory and move the robot synchronous in VREP
             */
            if (jsonHandler.get_op_mode() == OpMode::PTPSYNC) {
                Configuration start_cfg((jsonHandler.get_data())[0]);
                Configuration end_cfg((jsonHandler.get_data())[1]);
                //                cout << "Path Configuration" << endl;
                //                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
                //                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

                Trajectory *trajectory = ctrl.move_robot_ptp(&start_cfg, &end_cfg, true);
                for (Configuration *cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c,
                                           0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                           simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            /*
             * Compute a LIN trajectory and move the robot in VREP
             */
            if (jsonHandler.get_op_mode() == OpMode::LIN) {
                Configuration start_cfg((jsonHandler.get_data())[0]);
                Configuration end_cfg((jsonHandler.get_data())[1]);
//                cout << "Path Configuration" << endl;
//                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
//                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

                Trajectory *trajectory = ctrl.move_robot_lin(&start_cfg, &end_cfg);
                for (Configuration *cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c,
                                           0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                           simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            if (jsonHandler.get_op_mode() == OpMode::SPLINE_RESET) {

                cout << "SPLINE_RESET" << endl;
                spline_path.clear();
                // Keep first element free in case the robot moves while adding points.
                // In SPLINE_MOVE, it gets set to the current start point.
                spline_path.push_back(nullptr);
                // TODO: return signal
                /*simxSetStringSignal(ID, "returnsignal2",
                                    reinterpret_cast<const simxUChar *>("0"),
                                    1, simx_opmode_oneshot);*/
            }
            if (jsonHandler.get_op_mode() == OpMode::SPLINE_ADD) {
                cout << "SPLINE_ADD" << endl;
                SixDPos* next_pos = new SixDPos((jsonHandler.get_data())[0]);
                spline_path.push_back(next_pos);
                // TODO: return signal
                /*string size = to_string(spline_path.size() - 1);
                simxSetStringSignal(ID, "returnsignal2",
                                    reinterpret_cast<const simxUChar *>(size.c_str()),
                                    size.length(), simx_opmode_oneshot);*/
            }
            if (jsonHandler.get_op_mode() == OpMode::SPLINE_MOVE) {
                cout << "SPLINE_MOVE" << endl;
                SixDPos* start_pos = new SixDPos((jsonHandler.get_data())[0]);
                spline_path[0] = start_pos;
                Trajectory *trajectory = ctrl.move_robot_spline(spline_path);
                for (Configuration *cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c,
                                           0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                           simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            simxClearStringSignal(ID, "callsignal", simx_opmode_blocking);
            length = 0;
        }
    }
    simxFinish(ID);

    return 0;
}
