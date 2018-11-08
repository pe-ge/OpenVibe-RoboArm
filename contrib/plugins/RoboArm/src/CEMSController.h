#ifndef __CEMSController_H__
#define __CEMSController_H__

/**
    CEMSController.h
    Purpose: Controller for USB relay controlling EMS.

    @author Peter Gergel
    @version 1.0 16.10.2018
*/

#include <windows.h>

class CEMSController
{
	private:
		HANDLE hComPort;
		char cmdBuffer[32];
		char responseBuffer[32];
		DWORD numBytesWritten;
		DWORD numBytesRead;

		int FindPortNum();
	public:
		CEMSController();
		~CEMSController();

		char* relayRead();
		void relayOn();
		void relayOff();
};

#endif // __CEMSController_H__