#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "CRoboArmController.h"
#include "CRoboArmException.h"
#include "CEMSController.h"

#include <iostream>
#include <string>

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::RoboArm;

CBoxAlgorithmRoboArmStream::CBoxAlgorithmRoboArmStream(void)
	:m_ptRoboArm(NULL), m_ptCommunicationHandlerThread(NULL)
{
}

OpenViBE::boolean CBoxAlgorithmRoboArmStream::initialize ( void )
{
	m_oInput0Decoder.initialize(*this, 0);

	// Retrieve box settings
	m_bRoboArmConnected			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 0);
	m_ui64MovementSpeed			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1);
	m_ui64MovementSleep			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 2);
	m_ui64TopAngle				= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 3);
	m_ui64BottomAngle			= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 4);
	m_bEMSActive				= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 5);
	m_ui64EMSStimulationTime	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 6);

	if (m_ui64TopAngle < 0 || m_ui64TopAngle > 90 || m_ui64BottomAngle < 0 || m_ui64BottomAngle > 90)
	{
		this->getLogManager( ) << LogLevel_Error << "Incorrect angle values were set. Valid values are <0-90>.\n";
		return false;
	}
	this->getLogManager( ) << LogLevel_Info << "Movement speed was set:" << m_ui64MovementSpeed << "\n";
	this->getLogManager( ) << LogLevel_Info << "Top Angle was set:" << m_ui64TopAngle << "\n";
	this->getLogManager( ) << LogLevel_Info << "Bottom Angle was set:" << m_ui64BottomAngle << "\n";

	// Initialize robo arm
	if (m_bRoboArmConnected)
	{
		try
		{
			m_ptRoboArm = new CRoboArmController();
			if (!m_ptRoboArm->isRoboArmResponding())
			{
				this->getLogManager( ) << LogLevel_Error << "Robo arm is not responding.\n";
				return false;
			}
			m_ptRoboArm->setAngles(m_ui64TopAngle, m_ui64BottomAngle);
		} catch (const CRoboArmException &e)
		{
			this->getLogManager( ) << LogLevel_Error << e.what() << "\n";
			m_bRoboArmConnected = false;
			return false;
		}
	}

	// Initialize EMS
	if (m_bEMSActive)
	{
		try
		{
			m_ptEMS = new CEMSController();
			m_ptEMS->relayOff();
			std::cout << m_ptEMS->relayRead() << std::endl;
		} catch (const CRoboArmException &e)
		{
			this->getLogManager( ) << LogLevel_Warning << e.what() << "\n";
			m_bEMSActive = false;
		}
		
	}

	m_bSimulationRunning = true;
	m_bRecievedStartTrigger = false;
	m_bRecievedStopTrigger = false;
	m_ptCommunicationHandlerThread = new boost::thread( boost::bind( &CBoxAlgorithmRoboArmStream::CommunicationHandler, this ) );
	
	return true;
}

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
	if (m_bEMSActive)
	{
		if (m_ptEMS != NULL)
		{
			delete m_ptEMS;
			m_ptEMS = NULL;
		}
	}
	return true;
}

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	while (m_bSimulationRunning && m_bRoboArmConnected)
	{
		if (m_bRecievedStartTrigger)
		{
			int angleUp, angleDown;
			int position;

			// move robo arm UP
			m_ptRoboArm->continuousMovement(Direction::DIRECTION_UP, m_ui64MovementSpeed, m_ui64TopAngle / 5);

			// activate EMS
			if (m_bEMSActive)
			{
				m_ptEMS->relayOn();
				std::cout << m_ptEMS->relayRead() << std::endl;
				// EMS stimulation for defined period
				boost::this_thread::sleep(boost::posix_time::milliseconds(m_ui64EMSStimulationTime));
			}

			OpenViBE::uint64 robotSleepTime = m_ui64MovementSleep;
			// stop EMS
			if (m_bEMSActive)
			{
				m_ptEMS->relayOff();
				std::cout << m_ptEMS->relayRead() << std::endl;
				robotSleepTime -= m_ui64EMSStimulationTime;
				if (robotSleepTime < 0) robotSleepTime = 0;
			}

			// wait until m_ui64MovementSleep is finished
			boost::this_thread::sleep(boost::posix_time::milliseconds(robotSleepTime));

			m_ptRoboArm->getAngles(angleUp, angleDown);
			m_ptRoboArm->getPosition(position);

			// move robo arm DOWN
			m_ptRoboArm->continuousMovement(Direction::DIRECTION_DOWN, m_ui64MovementSpeed, m_ui64TopAngle / 5 + 1);
			// wait until m_ui64MovementSleep is finished
			boost::this_thread::sleep(boost::posix_time::seconds(m_ui64MovementSleep));

			m_ptRoboArm->getAngles(angleUp, angleDown);
			m_ptRoboArm->getPosition(position);

			m_bRecievedStartTrigger = false;
		}
	}
}

OpenViBE::boolean CBoxAlgorithmRoboArmStream::processInput(uint32 ui32InputIndex)
{
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();

	return true;
}

OpenViBE::boolean CBoxAlgorithmRoboArmStream::process (void)
{
	IBoxIO& l_rDynamicBoxContext = this->getDynamicBoxContext();

	for (uint32 i = 0; i < l_rDynamicBoxContext.getInputChunkCount(0); i++)
	{
		m_oInput0Decoder.decode(0, true);

		if (m_oInput0Decoder.isBufferReceived())
		{
			IStimulationSet* l_pStimulations = m_oInput0Decoder.getOutputStimulationSet();
			for (uint32 j = 0; j < l_pStimulations->getStimulationCount(); j++)
			{
				uint64 l_ui64StimulationCode = l_pStimulations->getStimulationIdentifier(j);
				if (l_ui64StimulationCode == OVTK_StimulationId_SegmentStart)
				{
					m_bRecievedStartTrigger = true;
				} else if (l_ui64StimulationCode == OVTK_StimulationId_SegmentStop)
				{
					m_bRecievedStopTrigger = true;
				}
			}
		}

	}
	
	return true;
}