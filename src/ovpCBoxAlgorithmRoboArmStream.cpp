#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <string>
#include <iostream>
#include <boost/thread.hpp>

#if defined TARGET_OS_Windows
	#include <windows.h>
	#define boolean OpenViBE::boolean
#endif
#if defined TARGET_OS_Linux
	#include <unistd.h>
#endif

#include <stdint.h>
#include "CRoboArmController.h"
#include "CRoboArmException.h"

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::RoboArm;

boolean CBoxAlgorithmRoboArmStream::initialize ( void )
{
	// Stimulation stream decoder
	m_oInput0Decoder.initialize(*this, 0);

	// Retrieving setting values
	m_ui64TopAngle		= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 0 );
	m_ui64BottomAngle	= FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1 );
	if (m_ui64TopAngle < 0 || m_ui64TopAngle > 90 || m_ui64BottomAngle < 0 || m_ui64BottomAngle > 90)
	{
		this->getLogManager( ) << LogLevel_Error << "Incorrect angle values were set. Valid values are <0-90>.\n";
		return false;
	}

	// Debug logs
	this->getLogManager( ) << LogLevel_Info << "Top Angle was set:" << m_ui64TopAngle << "\n";
	this->getLogManager( ) << LogLevel_Info << "Bottom Angle was set:" << m_ui64BottomAngle << "\n";

	try
	{
		m_ptRoboArm = new CRoboArmController();
	} catch (const CRoboArmException &e)
	{
		this->getLogManager( ) << LogLevel_Error << e.what() << "\n";
		return false;
	}

	// Strat reading thread ----------------------------------------------------------------------------------------------------------------------------------------
	m_bSimulationRunning = true;
	m_ptCommunicationHandlerThread = new boost::thread( boost::bind( &CBoxAlgorithmRoboArmStream::CommunicationHandler, this ) );

	return true;
}
//*******************************************************************************/

boolean CBoxAlgorithmRoboArmStream::uninitialize ( void )
{
	m_oInput0Decoder.uninitialize();

	m_bSimulationRunning = false;

	if (m_ptCommunicationHandlerThread != NULL)
	{
		m_ptCommunicationHandlerThread->interrupt( ); // Ask thread to stop
	}

	return true;
}

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	while ( m_bSimulationRunning )
	{

	}
}

/*******************************************************************************/
boolean CBoxAlgorithmRoboArmStream::processInput(uint32 ui32InputIndex)
{
	// some pre-processing code if needed...

	// ready to process !
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();

	return true;
}
/*******************************************************************************/
boolean CBoxAlgorithmRoboArmStream::processMessage(const IMessageWithData& msg, uint32 inputIndex)
{
	//If you know what the message should contain, you can directly access the values by using 
	//getters of the message class with known keys. Otherwise, you can loop over the contents to discover the keys.
	
	//You can get the first CString key of the message by calling this function
	//const CString *l_sKey = msg.getFirstCStringToken();
	//You can then go through all the keys by calling
	// getNextCStringToken(previousKey)
	//The function will return NULL when no more keys are found
#if 0
	while(l_sKey!=NULL)
	{
		l_sKey = msg.getNextCStringToken(*l_sKey);
		//and access the content with
		CString* l_sContent;
		boolean ok = msg.getValueCString(l_sKey, &l_sContent);
		//if ok is false, the retrieval was not successful
		//the message will be deleted when the function goes out of scope, store the value if you wish to use it later
	}
	
	//Same thing for the other types
	const CString *l_sMatrixKey = msg.getFirstIMatrixToken();
	while(l_sMatrixKey!=NULL)
	{
		l_sMatrixKey = msg.getNextIMatrixToken(*l_sMatrixKey);
		//and access the content with
		IMatrix* l_oContent;
		boolean ok = msg.getValueIMatrix(l_sMatrixKey, &l_oContent);
		//if ok is false, the retrieval was not successful
		//the message will be deleted when the function goes out of scope, store the value if you wish to use it later
		//for matrices, the copy is done that way
		//CMatrix * l_oLocalMatrix = new CMatrix();
		//OpenViBEToolkit::Tools::Matrix::copy(*l_oLocalMatrix, *l_oContent);
	}
#endif
	
	// Remember to return false in case the message was unexpected (user has made a wrong connection)	
	return true;
}
/*******************************************************************************/
boolean CBoxAlgorithmRoboArmStream::process ( void )
{
	// the static box context describes the box inputs, outputs, settings structures
	IBox& l_rStaticBoxContext = this->getStaticBoxContext();
	// the dynamic box context describes the current state of the box inputs and outputs (i.e. the chunks)
	IBoxIO& l_rDynamicBoxContext = this->getDynamicBoxContext( );

	return true;
}

