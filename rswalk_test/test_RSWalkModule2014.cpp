#include "RSWalkModule2014.h"

#include <iostream>
#include <unistd.h>
#include "zhelpers.hpp"

using namespace std;


static zmq::context_t context(1);
static zmq::socket_t publisher(context, ZMQ_PUB);
static zmq::socket_t subscriber (context, ZMQ_SUB);
static vector<double> control;
double * q2go;
static void sendStates(){
								std::ostringstream stream;
								stream << "robo_states|";
								std::cout<<"NUM_JOINTS "<<NUM_JOINTS<<std::endl;
								for (int i=0;i<NUM_JOINTS;i++){
									stream<<q2go[i]<<"|";
								}
								stream<< 0;
								cout<<(stream.str())<<endl;
								s_send (publisher, stream.str());
								printf("zmq: state sent \n");
}
void test_processFrame(RSWalkModule2014 myRS) {
								myRS.initSpecificModule();

								while (1) {

																myRS.processFrame();
																double size;
																myRS.getjointcommand(q2go,size);
																std::cout<<" Commanded joints ";
												        for (int q=0; q<size; q++) {
																	std::cout<<q2go[q]<<" ";
																}
																std::cout<<std::endl;
																sendStates();
																usleep(0.05*1000000);
								}

}

int main() {
								publisher.bind("tcp://*:5588"); /**/
								subscriber.connect("tcp://localhost:5589");
								subscriber.setsockopt( ZMQ_SUBSCRIBE, "control_msg", 1);
								printf("zmq setup \n");
								cout << "hello hello" << endl;
								// memset(q2go,0,NUM_JOINTS);
								q2go=(double*)std::malloc(NUM_JOINTS*sizeof(double));


								RSWalkModule2014 myRS;
								test_processFrame(myRS);

								return 0;
}
