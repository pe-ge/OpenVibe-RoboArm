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

namespace RoboArm
{
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
		DWORD EventDWord;
		char TxBuffer[64], RxBuffer[64];
		DWORD TxBytes, RxBytes;
		DWORD BytesSent, BytesReceived;

		// helpers
		bool send(const char * message);

	public:
		CRoboArmController(void);
		~CRoboArmController(void);
		
		bool isRoboArmResponding(void); // HW?
		bool getPosition(int& position); // COUNTER?
		bool getAngles(int& angleUp, int& angleDown); // ANGLES?
		bool executeMovement(Direction direction, int steps); // STEP
		bool startCyclicMovement(int speed); // START
		bool stopCyclicMovement(void); // STOP
		bool setAngles(int angleUp, int angleDown); // SET+ANGLE
		bool setDefaultPosition(); // CALIBRATION;
	};
};

#endif // __CRoboArmController_H__