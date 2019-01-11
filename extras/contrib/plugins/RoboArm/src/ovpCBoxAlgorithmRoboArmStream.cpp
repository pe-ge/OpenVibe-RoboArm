#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "CRoboArmController.h"
#include "CRoboArmException.h"

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::RoboArm;
/*******************************************************************************/

CBoxAlgorithmRoboArmStream::CBoxAlgorithmRoboArmStream(void)
{
    m_ptRoboArm = NULL;
    m_ptCommunicationHandlerThread = NULL;
	m_ptEMS = NULL;
}
/*******************************************************************************/

OpenViBE::boolean CBoxAlgorithmRoboArmStream::initialize ( void )
{
	m_oInput0Decoder.initialize(*this, 0);

	// Retrieve box settings
	m_ui32MovementSpeed			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 0);
	m_ui32MovementSleep			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1);
	m_ui32TopAngle				= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 2);
	m_ui32BottomAngle			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 3);
	m_ui32EMSStimulationTime	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 4);

	if (m_ui32TopAngle < 0 || m_ui32TopAngle > 90 || m_ui32BottomAngle < 0 || m_ui32BottomAngle > 90)
	{
		this->getLogManager( ) << LogLevel_Error << "Incorrect angle values were set. Valid values are <0-90>.\n";
		return false;
	}
	this->getLogManager( ) << LogLevel_Info << "Movement speed set to:" << m_ui32MovementSpeed << "\n";
	this->getLogManager( ) << LogLevel_Info << "Top Angle set to:" << m_ui32TopAngle << "\n";
	this->getLogManager( ) << LogLevel_Info << "Bottom Angle set to:" << m_ui32BottomAngle << "\n";

	// check whether robo arm is connected
	m_bRoboArmConnected = false;
	try
	{
		m_ptRoboArm = new CRoboArmController();
		if (!m_ptRoboArm->isRoboArmResponding())
		{
			throw CRoboArmException("Robo arm is not responding.");
		}
		m_ptRoboArm->setAngles(m_ui32TopAngle, m_ui32BottomAngle);
        m_bRoboArmConnected = true;
	} catch (const CRoboArmException &e)
	{
		this->getLogManager( ) << LogLevel_Warning << e.what() << "\n";
	}
    
    if (!m_bRoboArmConnected)
    {
        if (m_ptRoboArm != NULL)
		{
			delete m_ptRoboArm;
		}
		m_ptRoboArm = NULL;
    }

	// check whether EMS is connected
	m_bEMSConnected = false;
	try
	{
		m_ptEMS = new CEMSController();
		m_ptEMS->relayOff();
		std::cout << m_ptEMS->relayRead() << std::endl;
        m_bEMSConnected = true;
	} catch (const CRoboArmException &e)
	{
		this->getLogManager( ) << LogLevel_Warning << e.what() << "\n";
	}
    
    if (!m_bEMSConnected)
    {
        if (m_ptEMS != NULL)
		{
			delete m_ptEMS;
		}
		m_ptEMS = NULL;
    }

	m_bSimulationRunning = true;
	m_bRecievedStartTrigger = false;
	m_ptCommunicationHandlerThread = new boost::thread( boost::bind( &CBoxAlgorithmRoboArmStream::CommunicationHandler, this ) );
	
	return true;
}
/*******************************************************************************/

OpenViBE::boolean CBoxAlgorithmRoboArmStream::uninitialize ( void )
{
	m_oInput0Decoder.uninitialize();

	m_bSimulationRunning = false;

	if (m_ptCommunicationHandlerThread != NULL)
	{
		m_ptCommunicationHandlerThread->interrupt( ); // Ask thread to stop
		delete m_ptCommunicationHandlerThread;
		m_ptCommunicationHandlerThread = NULL;
	}
	if (m_bRoboArmConnected)
	{
		if (m_ptRoboArm != NULL)
		{
			delete m_ptRoboArm;
			m_ptRoboArm = NULL;
		}
	}
	if (m_bEMSConnected)
	{
		if (m_ptEMS != NULL)
		{
			delete m_ptEMS;
			m_ptEMS = NULL;
		}
	}
	return true;
}
/*******************************************************************************/

OpenViBE::boolean CBoxAlgorithmRoboArmStream::processInput(uint32 ui32InputIndex)
{
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();

	return true;
}
/*******************************************************************************/

OpenViBE::boolean CBoxAlgorithmRoboArmStream::process (void)
{
	IBoxIO& l_rDynamicBoxContext = this->getDynamicBoxContext();

	for (uint32 chunk = 0; chunk < l_rDynamicBoxContext.getInputChunkCount(0); chunk++)
	{
		m_oInput0Decoder.decode(chunk);

		if (m_oInput0Decoder.isBufferReceived())
		{
			IStimulationSet* l_pStimulations = m_oInput0Decoder.getOutputStimulationSet();
			for (uint64 j = 0; j < l_pStimulations->getStimulationCount(); j++)
			{
				uint64 l_ui32StimulationCode = l_pStimulations->getStimulationIdentifier(j);
				if (l_ui32StimulationCode == OVTK_StimulationId_SegmentStart)
				{
					m_bRecievedStartTrigger = true;
				}
			}
		}

	}
	
	return true;
}
/*******************************************************************************/

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{

	while (m_bSimulationRunning && m_bRoboArmConnected)
	{
		if (m_bRecievedStartTrigger)
		{
			// activate EMS
			if (m_bEMSConnected)
			{
				m_ptEMS->relayOn();
				std::cout << m_ptEMS->relayRead() << std::endl;
			}

			// move robo arm UP
			m_ptRoboArm->continuousMovement(Direction::DIRECTION_UP, m_ui32MovementSpeed, m_ui32TopAngle / 5);

			// stimulate for defined period
			OpenViBE::uint32 EMSSleepTime = 0;
			if (m_bEMSConnected) EMSSleepTime = m_ui32EMSStimulationTime;
			boost::this_thread::sleep(boost::posix_time::milliseconds(EMSSleepTime));
			
			OpenViBE::uint32 robotSleepTime = m_ui32MovementSleep;
			// stop EMS
			if (m_bEMSConnected)
			{
				m_ptEMS->relayOff();
				std::cout << m_ptEMS->relayRead() << std::endl;
				robotSleepTime -= m_ui32EMSStimulationTime;
				if (robotSleepTime < 0) robotSleepTime = 0;
			}

			// wait until m_ui32MovementSleep is finished
			boost::this_thread::sleep(boost::posix_time::milliseconds(robotSleepTime));

			unsigned int angleUp, angleDown;
			unsigned int position;
			m_ptRoboArm->getAngles(angleUp, angleDown);
			m_ptRoboArm->getPosition(position);

			// move robo arm DOWN
			m_ptRoboArm->continuousMovement(Direction::DIRECTION_DOWN, m_ui32MovementSpeed, m_ui32TopAngle / 5 + 1);
			// wait until m_ui32MovementSleep is finished
			boost::this_thread::sleep(boost::posix_time::milliseconds(m_ui32MovementSleep));

			m_ptRoboArm->getAngles(angleUp, angleDown);
			m_ptRoboArm->getPosition(position);

			m_bRecievedStartTrigger = false;
		}
	}
}