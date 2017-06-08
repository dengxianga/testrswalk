#include <Walk2014Generator.hpp>
#include <iostream>
using namespace std;

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
	
	ActionCommand::All request(head, body, leds, sonar);
	Odometry odometry(0.0f, 0.0f, 0.0f);
	SensorValues sensors;
	BodyModel bodyModel;
	float ballX = 0.0f;
	float ballY = 0.0f;

	JointValues result = myWalk.makeJoints(&request, 
						&odometry, 
						sensors, 
						bodyModel,
						ballX,
						ballY);
	
	cout << "makeJoints result :  " << result.temperatures[0] << endl;
}

int main() {
	cout<<"START"<<endl;

	Walk2014Generator myWalk;
	

	cout << "DONE" << endl;

	return 0;
}
// Walk2014Generator instance = Walk2014Generator.initilise();
