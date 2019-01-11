#ifndef __CRoboArmController_H__
#define __CRoboArmController_H__

/**
    CRoboArmController.h
    Purpose: Handles communcation with robotic device.

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
		char TxBuffer[64], RxBuffer[64];
		DWORD TxBytes, RxBytes;
		DWORD BytesSent, BytesReceived;
		DWORD EventDWord;
	public:
		CRoboArmController();
		~CRoboArmController();

		bool send(const char * message);
		bool receive();
		
		bool isRoboArmResponding(); // HW?
		bool getPosition(unsigned int& position); // COUNTER?
		bool getAngles(unsigned int& angleUp, unsigned int& angleDown); // ANGLES?
		bool step(Direction direction, int steps); // STEP
		bool startCyclicMovement(int speed); // START
		bool stopCyclicMovement(); // STOP
		bool setAngles(unsigned int angleUp, unsigned int angleDown); // SET+ANGLE
		bool calibrate(); // CALIBRATION;
		bool continuousMovement(Direction direction, unsigned int speed, unsigned int steps); // doNSteps
};

#endif // __CRoboArmController_H__