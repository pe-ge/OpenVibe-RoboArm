#include "CRoboArmException.h"

CRoboArmException::CRoboArmException(const std::string& message)
	: std::runtime_error(message)
{
};