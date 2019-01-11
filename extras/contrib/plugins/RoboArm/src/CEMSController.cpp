#include "CEMSController.h"
#include "CRoboArmException.h"
#include <sstream>
#include <iostream>

CEMSController::CEMSController()
{
	// Find correct port number
	int portNum = FindPortNum();
	std::stringstream sstmA;
	sstmA << "\\\\.\\COM" << portNum;
	std::string portName = sstmA.str();

	// Open port
	hComPort = CreateFile(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

	if (hComPort == INVALID_HANDLE_VALUE)
	{
		throw CRoboArmException("EMS error: Unable to open the specified port");
	}
};

CEMSController::~CEMSController()
{
	CloseHandle(hComPort);
};

int CEMSController::FindPortNum() //added function to find the present serial 
{
    TCHAR lpTargetPath[5000]; // buffer to store the path of the COMPORTS
	
	// iterate over ports COM0 - COM255
	std::stringstream sstm;
	for (unsigned int portNum = 0; portNum < 255; portNum++)
	{
		sstm.str("");
		sstm << "COM" << portNum;
		std::string portName = sstm.str();
		DWORD test = QueryDosDevice(portName.c_str(), lpTargetPath, 5000);

		if (test == 19)  // QueryDosDevice returns 19 for USB relay, I know its dirty :-(
		{
			return portNum;
		}
	}

	throw CRoboArmException("EMS error: Unable to open the specified port");
}

char* CEMSController::relayRead()
{
	cmdBuffer[0] = 0x0D;
	
	if(!WriteFile(hComPort, cmdBuffer, 1, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("Ems error: Unable to write to the specified port\n");
	}

	/* Flush the Serial port's RX buffer. This is a very important step*/
	Sleep(10);
	PurgeComm(hComPort, PURGE_RXCLEAR|PURGE_RXABORT);

	/* Copy the command to the command buffer */
	strcpy_s(cmdBuffer, "relay read 0");

	/* Append 0x0D to emulate ENTER key */
	cmdBuffer[12] = 0x0D;
	
	/* Write the command to the relay module. Total 13 bytes including 0x0D  */

	if(!WriteFile(hComPort, cmdBuffer, 13, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to write to the specified port");
	}

	/*Read back the response*/
	if(!ReadFile(hComPort, responseBuffer, 17, &numBytesRead, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to read from the specified port");
	}

	/* Add a null character at the end of the response so we can use the buffer
	   with string manipulation functions.
	 */
	responseBuffer[numBytesRead] = '\0';

	return responseBuffer + 14;
}

void CEMSController::relayOn()
{
	/* Copy the command to the command buffer */
	strcpy_s(cmdBuffer, "relay on 0");

	/* Append 0x0D to emulate ENTER key */
	cmdBuffer[10] = 0x0D;
	
	/* Write the command to the relay module. Total 11 bytes including 0x0D  */

	if(!WriteFile(hComPort, cmdBuffer, 11, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to write to the specified port\n");
	}
}

void CEMSController::relayOff()
{
	cmdBuffer[0] = 0x0D;
	
	if(!WriteFile(hComPort, cmdBuffer, 1, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to write to the specified port\n");
	}

	/* Copy the command to the command buffer */
	strcpy_s(cmdBuffer, "relay off 0");

	/* Append 0x0D to emulate ENTER key */
	cmdBuffer[11] = 0x0D;
	
	/* Write the command to the relay module. Total 11 bytes including 0x0D  */

	if(!WriteFile(hComPort, cmdBuffer, 11, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to write to the specified port\n");
	}
}