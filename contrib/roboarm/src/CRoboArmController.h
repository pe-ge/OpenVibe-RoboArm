#ifndef __CRoboArmController_H__
#define __CRoboArmController_H__

/**
    CRoboArmController.h
    Purpose: Handles communcation with robotic arm.

    @author Peter Gergel
    @version 1.0 11.8.2015
*/

#include <windows.h>
#include "ftd2xx.h"

enum Direction
{
	DIRECTION_DOWN,
	DIRECTION_UP
};

class CRoboArmController
{
private:
	// FTDI-related members
	FT_HANDLE ftHandle;
	FT_STATUS ftStatus;
	HANDLE hEvent;
	char TxBuffer[64], RxBuffer[64];
	DWORD TxBytes, RxBytes;
	DWORD BytesSent, BytesReceived;
	DWORD EventDWord;
public:
	CRoboArmController();
	~CRoboArmController();

	bool send(const char * message);
	bool receive();
	void waitUntilMsgAvailable();
		
	bool isRoboArmResponding(); // HW?
	bool getPosition(int& position); // COUNTER?
	bool getAngles(int& angleUp, int& angleDown); // ANGLES?
	bool step(Direction direction, int steps); // STEP
	bool startCyclicMovement(int speed); // START
	bool stopCyclicMovement(); // STOP
	bool setAngles(int angleDown, int angleUp); // SET+ANGLE
	bool calibrate(); // CALIBRATION;
	bool continuousMovement(Direction direction, int speed, int steps); // doNSteps
};

#endif // __CRoboArmController_H__