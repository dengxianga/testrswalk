#include <Walk2014Generator.hpp>
#include <WalkEnginePreProcessor.hpp>
//#include <RSWalkModule2014.h> // error with flabuffers.h
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
	ActionCommand::All request;
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

void test_WalkEnginePreProcessor_Constructor(WalkEnginePreProcessor myPreProcessor) {
	cout<< "myPreProcessor.isActive():  " << myPreProcessor.isActive() << endl;
	myPreProcessor.reset();
	myPreProcessor.stop();
	cout << "myPreProcessor.isLinedUp():  " << myPreProcessor.isLinedUp() << endl;
	myPreProcessor.resetLinedUp();
	cout << "myPreProcessor.linedUp:  " << myPreProcessor.linedUp << endl;
	cout << "myPreProcessor.isKicking:  " << myPreProcessor.isKicking << endl;
	myPreProcessor.lineUpEngine;
}

void test_WalkEnginePreProcessor_LineUpEngine(WalkEnginePreProcessor myPreProcessor) {
	WalkEnginePreProcessor::LineUpEngine myLineUpEngine(myPreProcessor.walkEngine);
	cout << "myLineUpEngine.hasStarted:  " << myLineUpEngine.hasStarted << endl;
	myLineUpEngine.reset();
	myLineUpEngine.start(myLineUpEngine.foot);

	ActionCommand::All request;
	float ballX = 0.0f;
	float ballY = 0.0f;
	bool isSideKick = false;
	bool hasEnded = myLineUpEngine.hasEnded(&request, ballX, ballY, isSideKick);
	cout << "myLineUpEngine.hasEnded:  " << hasEnded << endl;
	myLineUpEngine.preProcess(&request, ballX, ballY);
	cout << "myLineUpEngine.linedUp:  " << myLineUpEngine.linedUp << endl;
	cout << "myLineUpEngine.linedUpCt:  " << myLineUpEngine.linedUpCt << endl;
}

void test_WalkEnginePreProcessor_DribbleEngine(WalkEnginePreProcessor myPreProcessor) {
	WalkEnginePreProcessor::DribbleEngine myDribbleEngine(myPreProcessor.walkEngine);
	cout << WalkEnginePreProcessor::DribbleEngine::TURN << endl;
	myDribbleEngine.reset();
	cout << "myDribbleEngine.hasEnded():  " << myDribbleEngine.hasEnded() << endl;

	myDribbleEngine.start(myDribbleEngine.foot);
	
	ActionCommand::All request;
	BodyModel bodyModel;
	myDribbleEngine.preProcess(&request, bodyModel);
}

//void test_RSWalkModule2014(RSWalkModule2014 myRSWalkModule) {
//	myRSWalkModule.specifyMemoryDependency();
//	myRSWalkModule.specifyMemoryBlocks();
//	myRSWalkModule.initSpecificModule();
//
//	ActionCommand::Body body;
//	myRSWalkModule.processTargetModeWalk(body);
//	myRSWalkModule.processWalkRequest(body);
//	myRSWalkModule.processFrame();
//	myRSWalkModule.readOptions("dummy");
//	myRSWalkModule.handleStepIntoKick();
//
//	
//	//myRSWalkModule.writeDataToFile();
//}

int main() {
	cout<<"hello world"<<endl;
	
	//Walk2014Generator myWalk;
	//test_Walk2014Generator(myWalk);

	//WalkEnginePreProcessor myPreProcessor;
	//test_WalkEnginePreProcessor_DribbleEngine(myPreProcessor);

	

	cout << "done" << endl;

	return 0;
}
// Walk2014Generator instance = Walk2014Generator.initilise();
