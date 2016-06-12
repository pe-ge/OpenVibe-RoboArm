#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "CRoboArmController.h"
#include "CRoboArmException.h"

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
	m_bRoboArmConnected = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 0 );
	m_ui64MovementSpeed	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1 );
	m_ui64TopAngle		= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 2 );
	m_ui64BottomAngle	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 3 );
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
			return false;
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
	return true;
}

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	while ( m_bSimulationRunning )
	{
		if (m_bRoboArmConnected)
		{
			if (m_bRecievedStartTrigger)
			{
				int angleUp, angleDown;
				int position;

				m_ptRoboArm->continuousMovement(Direction::DIRECTION_UP, m_ui64MovementSpeed, m_ui64TopAngle / 5);
				boost::this_thread::sleep(boost::posix_time::seconds(2));

				m_ptRoboArm->getAngles(angleUp, angleDown);
				m_ptRoboArm->getPosition(position);

				m_ptRoboArm->continuousMovement(Direction::DIRECTION_DOWN, m_ui64MovementSpeed, m_ui64TopAngle / 5 + 1);
				boost::this_thread::sleep(boost::posix_time::seconds(2));

				m_ptRoboArm->getAngles(angleUp, angleDown);
				m_ptRoboArm->getPosition(position);

				m_bRecievedStartTrigger = false;
			}

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