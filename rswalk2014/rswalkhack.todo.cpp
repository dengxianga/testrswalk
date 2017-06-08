#include "RSWalkModule2014.h"
#include "WalkEnginePreProcessor.hpp"
#include "Walk2014Generator.hpp"
#include "Walk2015Generator.hpp"
#include "DistributedGenerator.hpp"
#include "ClippedGenerator.hpp"

#include <math/Geometry.h>

// Runswift files
#include "types/JointValues.hpp"
#include "types/AbsCoord.hpp"
#include "perception/kinematics/Kinematics.hpp"
#include "BodyModel.hpp"



#include <common/Kicks.h>

#include "GyroConfig.h"

#define ODOMETRY_LAG 8

#define DEBUG_OUTPUT false
#define GSL_COLLECT_DATA false

enum FootSensorRegion
{
        left_front,
        left_back,
        left_right,
        left_left,
        right_front,
        right_back,
        right_right,
        right_left,
        none
};


/*-----------------------------------------------------------------------------
 * Motion thread tick function (copy from runswift MotionAdapter.cpp)
   This would be like processFrame() function as in our code
 *---------------------------------------------------------------------------*/
void RSWalkModule2014::processFrame() {


        // If we are changing commands we need to reset generators

        static WalkControl twalk = WALK_CONTROL_OFF;


        // Setting walk kick parameters in bodyModel
        bodyModel.walkKick = false;
        bodyModel.walkKickLeftLeg = false;
        bodyModel.walkKickHeading = 0.0f;

        // 1. Need odometry to send to WalkGenerator. makeJoints updates odometry for localization
        Odometry odo = Odometry(0.1,0,0);
        Odometry prev = Odometry(odo); // Another copy for after makeJoints call

        // 1.1 Get ball position relative to robot - this will be changed to a target position for line-up
        // This is for lining up to the ball
        double ballX;  double ballY;


        // 2. Convert our request to runswift request
        // 2.1 Head
        ActionCommand::Head head; // Use default head. Not using HeadGenerator

        // 2.2 Body
        ActionCommand::Body body;

//walk_request_->motion_ == WalkRequestBlock::WALK assumed
        if (true) {

                if(DEBUG_OUTPUT) cout << "Requested: WALK\n";
                body.actionType = ActionCommand::Body::WALK;
                body.forward = 0.1;
                body.left = 0;
                body.turn = 0;
                body.bend = 1;
                body.power = 1;
                body.speed = 0.25;
                body.isFast = false;

                if (bodyModel.walkKick and walk_request_->walk_control_status_ == WALK_CONTROL_SET) { //target_walk_active_) {
                        body.actionType = ActionCommand::Body::KICK;
                        body.speed = 1.0; //7.0 * DEG_T_RAD;
                        double ballX = 0.1;
                        double ballY = 0;
                        if (bodyModel.walkKickLeftLeg) body.foot = ActionCommand::Body::LEFT;
                        else body.foot = ActionCommand::Body::RIGHT;
                } else {
                        bodyModel.walkKick = false;
                }

        }



        // 2.3 LEDs and Sonar
        ActionCommand::LED leds; // Not important, use defaults - Josiah
        float sonar = 0.0;

        // Now we have everything we need for a runswift request
        ActionCommand::All request(head,body,leds,sonar);

        // 3. Create runswift sensor object
        SensorValues sensors;

        // 3.1 Sense Joints
        for (int ut_ind=0; ut_ind<NUM_JOINTS; ut_ind++) {
                if (ut_ind != RHipYawPitch) { // RS does not have this joint because RHYP is same as LHYP on Nao
                        int rs_ind = utJointToRSJoint[ut_ind];
                        sensors.joints.angles[rs_ind] = 0; // changed from raw_joints - Josiah
                        sensors.joints.temperatures[rs_ind] = 0;
                }
        }

        // Detect if legs are getting hot
        bodyModel.isLeftLegHot = false;
        bodyModel.isRightLegHot = false;

        // 3.2 Sensors
        for (int ut_ind=0; ut_ind<bumperRR + 1; ut_ind++) {
                int rs_ind = utSensorToRSSensor[ut_ind];
                sensors.sensors[rs_ind] = 0;
        }


        // 4. If robot is remaining still, calbrate x and y gyroscopes
        // 4.1 Calibrate gyroX:
        double cur_gyroX = sensors.sensors[RSSensors::InertialSensor_GyrX];
//        cout << cur_gyroX << " " << sensors.sensors[RSSensors::InertialSensor_GyrY] << " " << sensors.sensors[RSSensors::InertialSensor_AccX] << " " << sensors.sensors[RSSensors::InertialSensor_AccY] << endl;
        double delta_gyroX = abs(cur_gyroX - last_gyroX);
        // maintain moving averages for gyro and delta_gyro
        avg_gyroX = avg_gyroX * (1.0 - window_size) + cur_gyroX * window_size;
        avg_delta_gyroX = avg_delta_gyroX * (1.0- window_size) + delta_gyroX * window_size;
        //cout << "gyroX: " << cur_gyroX << endl;
        if (avg_delta_gyroX < delta_threshold) {
                // robot remains still, do calibration
                offsetX = avg_gyroX;
                // reset avg_delta so it does not keep recalibrating
                avg_delta_gyroX = reset;
                calX_count += 1;
                if (calX_count == 1) {
                        cout << "(First calibration, may not be accurate) A GyroX calibration was triggered, offsetX set to: " << offsetX << endl;
                }
                else {
                        cout << "A GyroX calibration was triggered, offsetX set to: " << offsetX << endl;
                }
                last_gyroX_time = frame_info_->seconds_since_start;
        }
        else {
                if (DEBUG_OUTPUT) cout << "avg_delta_gyroX is: " << avg_delta_gyroX <<  " GyroX not stable, no calibration" << endl;
        }

        last_gyroX = cur_gyroX;

        // 4.2 Calibrate gyroY:
        double cur_gyroY = sensors.sensors[RSSensors::InertialSensor_GyrY];
        double delta_gyroY = abs(cur_gyroY - last_gyroY);
        // maintain moving averages for gyro and delta_gyro
        avg_gyroY = avg_gyroY * (1.0 - window_size) + cur_gyroY * window_size;
        avg_delta_gyroY = avg_delta_gyroY * (1.0- window_size) + delta_gyroY * window_size;
        if (avg_delta_gyroY < delta_threshold) {
                // robot remains still, do calibration
                offsetY = avg_gyroY;
                // reset avg_delta so it does not keep recalibrating
                avg_delta_gyroY = reset;
                calY_count += 1;
                if (calY_count == 1) {
                        cout << "(First calibration, may not be accurate) A GyroY calibration was triggered, offsetY set to: " << offsetY << endl;
                }
                else {
                        cout << "A GyroY calibration was triggered, offsetY set to: " << offsetY << endl;
                }
                last_gyroY_time = frame_info_->seconds_since_start;
        }
        else {
                if (DEBUG_OUTPUT) cout << "avg_delta_gyroY is: " << avg_delta_gyroY <<  " GyroY not stable, no calibration" << endl;
        }

        last_gyroY = cur_gyroY;


        // After each gyro has completed two cycles it is calibrated.
        //TODO: not sure whether need to put calZ_count here, since calibrating gyroZ does not affect walk;
        if (calX_count >= 2)
                bodyModel.isGyroXCalibrated = true;
        if (calY_count >= 2)
                bodyModel.isGyroYCalibrated = true;
//	if (calZ_count >= 2)
//	  bodyModel.isGyroZCalibrated = true;



        // Apply offset and convert from rad/sec to rad /frame
        sensors.sensors[RSSensors::InertialSensor_GyrX] = (sensors.sensors[RSSensors::InertialSensor_GyrX] - offsetX) * 0.01;
        sensors.sensors[RSSensors::InertialSensor_GyrY] = (sensors.sensors[RSSensors::InertialSensor_GyrY] - offsetY) * 0.01;
        // V5s need to calibrate Z as well
        // cout << "gyroZ: " << sensors.sensors[RSSensors::InertialSensor_GyrRef] << endl;
        // sensors.sensors[RSSensors::InertialSensor_GyrRef] = (sensors.sensors[RSSensors::InertialSensor_GyrRef] - offsetZ) * 0.01;

        // 5. Prepare BodyModel to pass to makeJoints

        // 5.1 Get Kinematics ready for bodyModel. We should figure out what parameter values should be.
        kinematics.setSensorValues(sensors);



        kinematics.updateDHChain();




        // 5.2 update bodyModel
        bodyModel.kinematics = &kinematics;
        bodyModel.update(&odo, sensors);


        if (request.body.actionType == ActionCommand::Body::WALK && odometry_->walkDisabled) {
                static int delay_ct = 0;
                if (delay_ct % 100 == 0) //prev_command != walk_request_->motion_)
                        speech_->say("Not Calibrated");
                delay_ct++;
                if(DEBUG_OUTPUT) cout << "Not calibrated" << endl;
                if (odometry_->standing)
                        request.body = ActionCommand::Body::STAND;
                else
                        request.body = ActionCommand::Body::NONE;
        }

        if(DEBUG_OUTPUT) printf("motion request: %s, prev: %s, body request: %i\n", WalkRequestBlock::getName(walk_request_->motion_), WalkRequestBlock::getName(prev_command), static_cast<int>(request.body.actionType));
        prev_command = WalkRequestBlock::WALK;

        bool requestWalkKick = bodyModel.walkKick;
        // Call the clipped generator which calls the distributed generator to produce walks, stands, etc.
        JointValues joints = clipper->makeJoints(&request, &odo, sensors, bodyModel, ballX, ballY);


//	static int point_id_ = 0;
        if (GSL_COLLECT_DATA && request.body.actionType == ActionCommand::Body::WALK)
                writeDataToFile(sensors,joints);
//	point_id_ ++;

        // We aren't setting arms any more so this isn't important unless we go back to it
        // Setting arms behind back makes us get caught less but then we can't counter balance rswalk
        if ((frame_info_->seconds_since_start - last_walk_or_stand_) > 0.3) {
                arm_state_ = -1;
        }

        // Update odometry
        static double cum_f=0, cum_l=0, cum_t=0;

        Odometry delta = Odometry(odo - prev);
        cum_f += delta.forward;
        cum_l += delta.left;
        cum_t += delta.turn;
        // For debugging odometry
/*	if (body.turn != 0){
    cout << "Odometry Prev: " << prev.turn;
    cout << " Odometry Delta: " << delta.turn;
    cout << " Odometry: "  << odo.turn;
    cout << " Cumulative Odometry: " << cum_t << endl;
   }
 */

        // Update walk_info_


        // Update
        wasKicking =false;

        if (request.body.actionType == ActionCommand::Body::ActionType::NONE) {
                return;
        }

        // For setting arms
        last_walk_or_stand_ = 9999999;

        // Convert RS joints to UT joints and write to commands memory block

        // Ruohan: setting head stiffness was commented out in bhuman. Should ask Jake about this
        // Jake: setting stiffness here is unnecessary and means we can never turn these off for testing.
        //commands_->stiffness_[HeadPitch] = 1.0;
        //commands_->stiffness_[HeadYaw] = 1.0;


        // Walk keeps falling backwards when knees over 100. Leaning forward helps some


        // if the robot has walked, listen to arm command from WalkGenerator, else do stiffness

        selectivelySendStiffness(); // Only send stiffness if at least one joint has changed stiffness values by at least 0.01
//	setArms(commands_->angles_,0.01); // Arms getting stuck on robot front with rswalk. Needs tuning


        // Not doing anything with these now. Could be used in future for smoothing
        prevForward = body.forward;
        prevLeft = body.left;
        prevTurn = body.turn;

        // Agent Effector code?
        // This is in rswalk agent effecter. Not sure what standing is but it works now.
        static bool kill_standing = false;
        standing = kill_standing;
        if (kill_standing) {
                kill_standing = false;
                standing = false;
        }
        else{
                kill_standing = true;
        }

}

void RSWalkModule2014::readOptions(std::string path)
{

        clipper->readOptions(path);

}

RSWalkModule2014::RSWalkModule2014() :
/*    slow_stand_start(-1),
    slow_stand_end(-1),
    walk_requested_start_time(-1),
    prev_kick_active_(false), */
        arms_close_to_targets_(false),
        arm_state_(-1),
        arm_state_change_(-1),
        last_walk_or_stand_(-1),
        step_into_kick_state_(NONE),
        time_step_into_kick_finished_(0),
        walk_kick_finish_frame_(-100),
        last_gyroY_time(-1),
        last_gyroX_time(-1),
        prevForward(0),
        prevLeft(0),
        prevTurn(0)
{
        utJointToRSJoint[HeadYaw] = RSJoints::HeadYaw;
        utJointToRSJoint[HeadPitch] = RSJoints::HeadPitch;

        utJointToRSJoint[LShoulderPitch] = RSJoints::LShoulderPitch;
        utJointToRSJoint[LShoulderRoll] = RSJoints::LShoulderRoll;
        utJointToRSJoint[LElbowYaw] = RSJoints::LElbowYaw;
        utJointToRSJoint[LElbowRoll] = RSJoints::LElbowRoll;

        utJointToRSJoint[RShoulderPitch] = RSJoints::RShoulderPitch;
        utJointToRSJoint[RShoulderRoll] = RSJoints::RShoulderRoll;
        utJointToRSJoint[RElbowYaw] = RSJoints::RElbowYaw;
        utJointToRSJoint[RElbowRoll] = RSJoints::RElbowRoll;

        utJointToRSJoint[LHipYawPitch] = RSJoints::LHipYawPitch;
        utJointToRSJoint[LHipRoll] = RSJoints::LHipRoll;
        utJointToRSJoint[LHipPitch] = RSJoints::LHipPitch;
        utJointToRSJoint[LKneePitch] = RSJoints::LKneePitch;
        utJointToRSJoint[LAnklePitch] = RSJoints::LAnklePitch;
        utJointToRSJoint[LAnkleRoll] = RSJoints::LAnkleRoll;

        utJointToRSJoint[RHipYawPitch] = -1; //RSJoints::RHipYawPitch;
        utJointToRSJoint[RHipRoll] = RSJoints::RHipRoll;
        utJointToRSJoint[RHipPitch] = RSJoints::RHipPitch;
        utJointToRSJoint[RKneePitch] = RSJoints::RKneePitch;
        utJointToRSJoint[RAnklePitch] = RSJoints::RAnklePitch;
        utJointToRSJoint[RAnkleRoll] = RSJoints::RAnkleRoll;

        // SENSOR VALUE CONVERSION
        utSensorToRSSensor[gyroX] = RSSensors::InertialSensor_GyrX;
        utSensorToRSSensor[gyroY] = RSSensors::InertialSensor_GyrY;
        utSensorToRSSensor[gyroZ] = RSSensors::InertialSensor_GyrRef;
        utSensorToRSSensor[accelX] = RSSensors::InertialSensor_AccX;
        utSensorToRSSensor[accelY] = RSSensors::InertialSensor_AccY;
        utSensorToRSSensor[accelZ] = RSSensors::InertialSensor_AccZ;
        utSensorToRSSensor[angleX] = RSSensors::InertialSensor_AngleX;
        utSensorToRSSensor[angleY] = RSSensors::InertialSensor_AngleY;
        utSensorToRSSensor[angleZ] = RSSensors::InertialSensor_AngleZ;
        utSensorToRSSensor[battery] = RSSensors::Battery_Current;
        utSensorToRSSensor[fsrLFL] = RSSensors::LFoot_FSR_FrontLeft;
        utSensorToRSSensor[fsrLFR] = RSSensors::LFoot_FSR_FrontRight;
        utSensorToRSSensor[fsrLRL] = RSSensors::LFoot_FSR_RearLeft;
        utSensorToRSSensor[fsrLRR] = RSSensors::LFoot_FSR_RearRight;
        utSensorToRSSensor[fsrRFL] = RSSensors::RFoot_FSR_FrontLeft;
        utSensorToRSSensor[fsrRFR] = RSSensors::RFoot_FSR_FrontRight;
        utSensorToRSSensor[fsrRRL] = RSSensors::RFoot_FSR_RearLeft;
        utSensorToRSSensor[fsrRRR] = RSSensors::RFoot_FSR_RearRight;
        utSensorToRSSensor[centerButton] = RSSensors::ChestBoard_Button;
        utSensorToRSSensor[bumperLL] = RSSensors::LFoot_Bumper_Left;
        utSensorToRSSensor[bumperLR] = RSSensors::LFoot_Bumper_Right;
        utSensorToRSSensor[bumperRL] = RSSensors::RFoot_Bumper_Left;
        utSensorToRSSensor[bumperRR] = RSSensors::RFoot_Bumper_Right;
        // No UT Sensors for RSSensors RFoot_FSR_CenterOfPressure_X/Y, Battery_Charge or US, not sure what these are - Josiah


}

RSWalkModule2014::~RSWalkModule2014() {
}



void RSWalkModule2014::initSpecificModule() {
        // For RSWalk
        generator = new WalkEnginePreProcessor();
        std::string config_path = "";
        config_path += "/config/rswalk2014";
        clipper = new ClippedGenerator((Generator*) new DistributedGenerator(config_path));
        readOptions(config_path);
        standing = false;
        prev_command = WalkRequestBlock::NONE;
        x_target = -1;
        y_target = -1;
        wasKicking = false;



}



const float RSWalkModule2014::STAND_ANGLES[NUM_JOINTS] = {
        0,
        -0.366519,
        0,
        0.00669175,
        -0.548284,
        1.04734,
        -0.499061,
        -0.00669175,
        0,
        -0.00669175,
        -0.548284,
        1.04734,
        -0.499061,
        0.00669175,
        -1.5708,
        0.2,
        -1.5708,
        -0.2,
        -1.5708,
        0.2,
        -1.5708,
        -0.2
};

void RSWalkModule2014::selectivelySendStiffness() {
        for (int i = 0; i < NUM_JOINTS; i++) {
                if (fabs(joints_->stiffness_[i] - commands_->stiffness_[i]) > 0.01) {
                        commands_->send_stiffness_ = true;
                        commands_->stiffness_time_ = 10;
                        return;
                }
        }
}
