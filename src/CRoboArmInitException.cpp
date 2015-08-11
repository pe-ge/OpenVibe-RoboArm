#include "CRoboArmInitException.h"

using namespace RoboArm;

CRoboArmInitException::CRoboArmInitException(const std::string& message)
	: std::runtime_error(message)
{
};