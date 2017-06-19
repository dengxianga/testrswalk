#include "RSWalkModule2014.h"

#include <iostream>
#include <unistd.h>

using namespace std;

void test_processFrame(RSWalkModule2014 myRS) {
	myRS.initSpecificModule();

	while (1) {
		myRS.processFrame();
		sleep(1);
	}

}

int main() {
	cout << "hello hello" << endl;

	RSWalkModule2014 myRS;
	test_processFrame(myRS);

	return 0;
}
