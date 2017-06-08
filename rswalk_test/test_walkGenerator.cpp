#include <Walk2014Generator.hpp>
#include <WalkCycle.hpp>
#include <iostream>
#include <unistd.h>
using namespace std;

enum Sensor {
  gyroX,
  gyroY,
  gyroZ,
  accelX,
  accelY,
  accelZ,
  angleX,
  angleY,
  angleZ,
  battery,
  fsrLFL,
  fsrLFR,
  fsrLRL,
  fsrLRR,
  fsrRFL,
  fsrRFR,
  fsrRRL,
  fsrRRR,
  bumperLL,
  bumperLR,
  bumperRL,
  bumperRR,
  centerButton,
  headFront,
  headMiddle,
  headRear,
  NUM_SENSORS
};

enum Joint {
  HeadYaw,
  HeadPitch,
  LHipYawPitch,
  LHipRoll,
  LHipPitch,
  LKneePitch,
  LAnklePitch,
  LAnkleRoll,
  RHipYawPitch,
  RHipRoll,
  RHipPitch,
  RKneePitch,
  RAnklePitch,
  RAnkleRoll,
  LShoulderPitch,
  LShoulderRoll,
  LElbowYaw,
  LElbowRoll,
  RShoulderPitch,
  RShoulderRoll,
  RElbowYaw,
  RElbowRoll,
  NUM_JOINTS
};

void test_Walk2014Generator(Walk2014Generator myWalk) {
	cout << "myWalk.isActive():  " << myWalk.isActive() << endl;
	cout << "myWalk.isStanding():  " << myWalk.isStanding() << endl;
	myWalk.reset();
	myWalk.stop();
	myWalk.readOptions("dummy"); // does nothing
}

void test_makeJoints(Walk2014Generator myWalk) {
//TODO: fix result.angles and result.temperatures to be not "nan"
	
	ActionCommand::Head head;

	ActionCommand::Body body;

	body.actionType = ActionCommand::Body::STAND; 
	body.forward = 0;
	body.left = 0;
	body.turn = 0;
	body.isFast = false;
	body.bend = 1;
	body.power = 1;

	ActionCommand::LED leds;

	float sonar = 0.0;

	ActionCommand::All request(head, body, leds, sonar);

	Odometry odo = Odometry(0.5f, 0.5f, 0.1f); //x, y, rotation

	SensorValues sensors;

	for (int ut_ind=0; ut_ind<NUM_JOINTS; ut_ind++){
		if (ut_ind != RHipYawPitch){ // RS does not have this joint because RHYP is same as LHYP on Nao
			// Conversion not needed for testing
			// int rs_ind = utJointToRSJoint[ut_ind];

			//TODO: figure out joints_ and robot_joint_signs (something related to Module.h)
			//sensors.joints.angles[ut_ind] = joints_->values_[ut_ind] * robot_joint_signs[ut_ind]; // changed from raw_joints - Josiah
			//sensors.joints.temperatures[ut_ind] = sensors_->joint_temperatures_[ut_ind];
			sensors.joints.angles[ut_ind] = ut_ind;
			sensors.joints.temperatures[ut_ind] = ut_ind * 10;
		}
	}

	BodyModel bodyModel;

	Kinematics kinematics;

	kinematics.setSensorValues(sensors);
	kinematics.updateDHChain();  // 5.2 update bodyModel

	bodyModel.kinematics = &kinematics;
	bodyModel.update(&odo, sensors);

	// 3.2 Sensors
    for (int ut_ind=0; ut_ind<bumperRR + 1; ut_ind++){
        //Conversion not needed for testing
        //int rs_ind = utSensorToRSSensor[ut_ind];
        //sensors.sensors[ut_ind] = sensors_->values_[ut_ind];
        sensors.sensors[ut_ind] = ut_ind * 0.05f;
    }

	float ballX = 0.0f;
	float ballY = 0.0f;

	JointValues result = myWalk.makeJoints(&request, 
						&odo, 
						sensors, 
						bodyModel,
						ballX,
						ballY);

	// result.angles
	// result.temperatures
	// result.stiffnesses

	for (int i=0; i<NUM_JOINTS; i++) {
		cout << i << ":    " << result.angles[i] << endl;
	}
}

void test_WalkCycle(WalkCycle myWC) {
	//cout << "myWalkCycle.useForwardL:  " << myWalkCycle.useForwardL << endl;
	float forwardL, forwardR, leftL, leftR, turnLR, liftL, liftR;
	forwardL = 1.0f;
	while (1) {
		cout << "t: " << myWC.t << "   T: " << myWC.T << endl;

		cout << myWC.useForwardL << ", ";
		cout << myWC.useForwardR << ", ";
		cout << myWC.useLeft << ", ";
		cout << myWC.useTurn << endl;

		cout << forwardL << ", ";
		cout << forwardR << ", ";
		cout << leftL << ", ";
		cout << leftR << ", ";
		cout << turnLR << ", ";
		cout << liftL << ", ";
		cout << liftR << "\n" << endl;
		sleep(1);

		myWC.generateWalk(forwardL, forwardR, leftL, leftR, turnLR, liftL, liftR);
		myWC.next();
	}
//		unix.usleep(100000);
//		myWC.generateWalk(myWC.forwardL,
//					myWC.forwardR,
//					myWC.leftL,
//					myWC.leftR,
//					myWC.turnLR,
//					myWC.liftL,
//					myWC.liftR);


}

int main() {
	cout<<"START"<<endl;

	//Walk2014Generator myWalk;
	//test_makeJoints(myWalk);

	WalkCycle myWalkCycle(1.0f, 0.0f, 1.0f, 0.0f, 0.5f, 0.0f);
	test_WalkCycle(myWalkCycle);

	cout << "DONE" << endl;

	return 0;
}
// Walk2014Generator instance = Walk2014Generator.initilise();
