#include "CEMSController.h"
#include "CRoboArmException.h"

CEMSController::CEMSController(LPCSTR PortName)
{
	/*
		Open a handle to the COM port. We need the handle to send commands and
		receive results.
	*/

	hComPort = CreateFile(PortName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

	if (hComPort == INVALID_HANDLE_VALUE)
	{
		throw CRoboArmException("EMS error: Unable to open the specified port");
	}
};

CEMSController::~CEMSController()
{
	CloseHandle(hComPort);
};

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
	strcpy(cmdBuffer, "relay read 0");

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
	strcpy(cmdBuffer, "relay on 0");

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
	strcpy(cmdBuffer, "relay off 0");

	/* Append 0x0D to emulate ENTER key */
	cmdBuffer[11] = 0x0D;
	
	/* Write the command to the relay module. Total 11 bytes including 0x0D  */

	if(!WriteFile(hComPort, cmdBuffer, 11, &numBytesWritten, NULL))
	{
		CloseHandle(hComPort);
		throw CRoboArmException("EMS error: Unable to write to the specified port\n");
	}
}