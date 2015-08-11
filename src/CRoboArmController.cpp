#include "CRoboArmController.h"
#include "CRoboArmInitException.h"
#include "ftd2xx.h"
#include <iostream>

using namespace RoboArm;

bool CRoboArmController::send(const char * message)
{
	ftStatus = FT_OK;
	TxBytes = 0;
	RxBytes = 0;
	BytesSent = 0;
	BytesReceived = 0;

	// Send message
	TxBytes = sprintf(TxBuffer, "%s", message);
	ftStatus |= FT_Write(ftHandle, TxBuffer, TxBytes, &BytesSent);
	// Give robotic arm time to process message - 50ms is enough
	Sleep(150);
	// Obtain number of characters to be read;
	ftStatus |= FT_GetStatus(ftHandle,&RxBytes,&TxBytes,&EventDWord);
	// Read available data
	ftStatus |= FT_Read(ftHandle, RxBuffer, RxBytes, &BytesReceived);
	// Append NULL character to the end
	RxBuffer[RxBytes] = NULL;

	return ftStatus == FT_OK && RxBytes == BytesReceived;
}

void CRoboArmController::processErrorMessage()
{
}

bool CRoboArmController::processResponse(int response)
{
	if (response == 0)
	{
		return true;
	} else
	{
		processErrorMessage();
		return false;
	}
}

CRoboArmController::CRoboArmController( void )
{
	// Open the device
	ftStatus = FT_OpenEx("AH02QXEY", FT_OPEN_BY_SERIAL_NUMBER, &ftHandle); 
	if (ftStatus != FT_OK)
	{
		throw CRoboArmInitException("Robo arm is not connected");
	}

	// Reset the device
	ftStatus = FT_ResetDevice(ftHandle);
	// Purge transmit and receive buffers
	ftStatus = FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
	// Set the baud rate
	ftStatus = FT_SetBaudRate(ftHandle, 57600);
	// Set to communicate at 8N1
	ftStatus = FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
	// 300ms timeouts on read / write
	ftStatus = FT_SetTimeouts(ftHandle, FT_DEFAULT_RX_TIMEOUT + 2000, FT_DEFAULT_TX_TIMEOUT + 2000);
	// Disable hardware / software flow control
	ftStatus = FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
};

CRoboArmController::~CRoboArmController( void )
{
	// Close the device
	ftStatus = FT_Close(ftHandle);
	ftHandle = 0;
};

bool CRoboArmController::isRoboArmResponding(void)
{
	if (!send("HW?\r"))
	{
		return false;
	}

	int response = strcmp(RxBuffer, "HW?\r\rHW::ok\r");
	return processResponse(response);
}
bool CRoboArmController::getPosition(int& roboArmPosition)
{
	//if (!send("START=10\r"))
	if (!send("CALIBRATION\r"))
	{
		return false;
	}

	// Obtain number of characters to be read;
	ftStatus |= FT_GetStatus(ftHandle,&RxBytes,&TxBytes,&EventDWord);
	// Read available data
	ftStatus |= FT_Read(ftHandle, RxBuffer, RxBytes, &BytesReceived);

	std::cout << RxBuffer << std::endl;
	int response = strcmp(RxBuffer, "HW?\r\rHW::ok\r");
	return processResponse(response);
}
bool CRoboArmController::getAngles(int& upperAngle, int& bottomAngle)
{
	return true;
}
bool CRoboArmController::executeMovement(int direction, int steps)
{
	return true;
}
bool CRoboArmController::startCyclicMovement(int speed)
{
	return true;
}
bool CRoboArmController::stopCyclicMovement(void)
{
	return true;
}
bool CRoboArmController::setAngles(int upperAngle, int bottomAngle)
{
	return true;
}
bool CRoboArmController::setDefaultPosition()
{
	return true;
}