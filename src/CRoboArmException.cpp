#include "CRoboArmException.h"

using namespace RoboArm;

CRoboArmException::CRoboArmException(const std::string& message)
	: std::runtime_error(message)
{
};