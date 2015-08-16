#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <boost/thread.hpp>
#include "CRoboArmController.h"
#include "CRoboArmException.h"

#define ROBO_ARM_CONNECTED 0

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
	m_ui64TopAngle		= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 0 );
	m_ui64BottomAngle	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1 );
	if (m_ui64TopAngle < 0 || m_ui64TopAngle > 90 || m_ui64BottomAngle < 0 || m_ui64BottomAngle > 90)
	{
		this->getLogManager( ) << LogLevel_Error << "Incorrect angle values were set. Valid values are <0-90>.\n";
		return false;
	}
	this->getLogManager( ) << LogLevel_Info << "Top Angle was set:" << m_ui64TopAngle << "\n";
	this->getLogManager( ) << LogLevel_Info << "Bottom Angle was set:" << m_ui64BottomAngle << "\n";

	// Initialize robo arm
#if ROBO_ARM_CONNECTED
	try
	{
		m_ptRoboArm = new CRoboArmController();
		if (!m_ptRoboArm->isRoboArmResponding())
		{
			this->getLogManager( ) << LogLevel_Error << "Robo arm is not responding.\n";
			return false;
		}
	} catch (const CRoboArmException &e)
	{
		this->getLogManager( ) << LogLevel_Error << e.what() << "\n";
		return false;
	}
#endif

	m_bSimulationRunning = true;
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
#if ROBO_ARM_CONNECTED
	if (m_ptRoboArm != NULL)
	{
		delete m_ptRoboArm;
		m_ptRoboArm = NULL;
	}
#endif
	return true;
}

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	while ( m_bSimulationRunning )
	{
		
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
				uint64 l_ui64StimulationDate = l_pStimulations->getStimulationDate(j);
				CString l_sStimulationName = this->getTypeManager().getEnumerationEntryNameFromValue(OV_TypeId_Stimulation, l_ui64StimulationCode);
				if (l_ui64StimulationCode == OVTK_StimulationId_SegmentStart)
				{
					this->getLogManager() << LogLevel_Info << "Starting new try sequence.\n";
				}
				else if (l_ui64StimulationCode == OVTK_StimulationId_SegmentStop)
				{
					this->getLogManager() << LogLevel_Info << "Stopoing try sequence.\n";
				}
				else
				{
					this->getLogManager() << LogLevel_Warning << "Received an unrecognized trigger, with code [" << l_ui64StimulationCode
						<< "], name [" << l_sStimulationName << "] and date [" << time64(l_ui64StimulationDate) << "]\n";
				}
			}
		}

	}
	
	return true;
}