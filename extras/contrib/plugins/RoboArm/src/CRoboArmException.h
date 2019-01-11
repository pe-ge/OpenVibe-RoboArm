#ifndef __CRoboArmException_H__
#define ___CRoboArmException_H__

/**
    CRoboArmException.h
    Purpose: Exception thrown during manipulation with robotic arm.

    @author Peter Gergel
    @version 1.0 11.8.2015
*/

#include <stdexcept>

class CRoboArmException : public std::runtime_error
{
public:
	CRoboArmException(const std::string& message);
};

#endif // __CRoboArmException_H__