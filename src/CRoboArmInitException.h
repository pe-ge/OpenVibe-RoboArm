#ifndef __CRoboArmInitException_H__
#define ___CRoboArmInitException_H__

/**
    CRoboArmInitException.h.h
    Purpose: Exception thrown during failed initialization of robotic arm.

    @author Peter Gergel
    @version 1.0 11.8.2015
*/

#include <stdexcept>

namespace RoboArm
{
	class CRoboArmInitException : public std::runtime_error
	{
	public:
		CRoboArmInitException(const std::string& message);
	};
}

#endif // __CRoboArmInitException_H__