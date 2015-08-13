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

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::RoboArm;

using namespace boost;
using namespace boost::this_thread;

CBoxAlgorithmRoboArmStream::CBoxAlgorithmRoboArmStream( void ) :
	m_ptSerialCom(NULL)
{
};

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	while ( m_bContinueCommunication )
	{

	}
}



boolean CBoxAlgorithmRoboArmStream::initialize ( void )
{
	// Encoder & Decoders initializing -------------------------------------------------------------------------------------------------------------------------
	// Signal stream decoders
	m_oInput0Decoder.initialize(*this, 0);
	m_oInput1Decoder.initialize(*this, 1);
	// Stimulation stream decoder
	m_oInput2Decoder.initialize(*this, 2);
	// Signal stream encoder
	m_oOutput0Encoder.initialize(*this, 0);

	// We connect the Signal Input with the Signal Output so every chunk on the input will be copied to the output automatically.
	m_oOutput0Encoder.getInputMatrix().setReferenceTarget(m_oInput0Decoder.getOutputMatrix());
	m_oOutput0Encoder.getInputSamplingRate().setReferenceTarget(m_oInput0Decoder.getOutputSamplingRate());
	
	// If you need to, you can manually set the reference targets to link the codecs input and output. To do so, you can use :
	//m_oEncoder.getInputX().setReferenceTarget(m_oDecoder.getOutputX())
	// Where 'X' depends on the codec type. Please refer to the Codec Toolkit Reference Page
	// (http://openvibe.inria.fr/documentation/unstable/Doc_Tutorial_Developer_SignalProcessing_CodecToolkit_Ref.html) for a complete list.
	
	// Retrieving setting values -------------------------------------------------------------------------------------------------------------------------
	// Get Strategy Settings
	m_sStrategy = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 1 );
	// Get Manual Threshold value
	m_f64ThresholdValue = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 3 );
	// Get Threshold selection
	m_bUseThresholdSignal = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 2 );
	// Get Triggers settings 
	m_ui64StartTrigger = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 4 );
	m_ui64StopTrigger = FSettingValueAutoCast( *this->getBoxAlgorithmContext( ), 5 );

	// Debug logs -------------------------------------------------------------------------------------------------------------------------
	this->getLogManager( ) << LogLevel_Info << "Selected strategy:" << m_sStrategy << "\n";
	char buffer[128]; sprintf( buffer, "%.3f", m_f64ThresholdValue );
	this->getLogManager( ) << LogLevel_Info << "Manual Threshol value:" << buffer << "\n";
	this->getLogManager( ) << LogLevel_Info << "Start Trigger:" << this->getTypeManager( ).getEnumerationEntryNameFromValue( OV_TypeId_Stimulation, m_ui64StartTrigger ) << "\n";
	this->getLogManager( ) << LogLevel_Info << "Stop Trigger:" << this->getTypeManager( ).getEnumerationEntryNameFromValue( OV_TypeId_Stimulation, m_ui64StopTrigger ) << "\n";

	// Initializing  Triggers
	m_bStartSignalRequested = false;
	m_bStopSignalRequested = true;

	// Strat reading thread ----------------------------------------------------------------------------------------------------------------------------------------
	m_bContinueCommunication = true;
	m_ptSerialCom = new boost::thread( boost::bind( &CBoxAlgorithmRoboArmStream::CommunicationHandler, this ) );

	// END of FTDI initializing -----------------------------------------------------------------------------------------------------------------------------------

	return true;
}
//*******************************************************************************/

boolean CBoxAlgorithmRoboArmStream::uninitialize ( void )
{
	m_oInput0Decoder.uninitialize();
	m_oInput1Decoder.uninitialize();
	m_oInput2Decoder.uninitialize();
	m_oOutput0Encoder.uninitialize();

	m_bContinueCommunication = false;

	if (m_ptSerialCom != NULL)
	{
		m_ptSerialCom->interrupt( ); // Ask thread to stop
	}

	return true;
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

	// we will first check the pending stimulations to check if the trigger has been received.
	// iterate over all chunk on input 2 (Stimulations)
	for ( uint32 i = 0; i < l_rDynamicBoxContext.getInputChunkCount( 2 ); i++ )
	{
		//we decode the ith chunk on input 2
		m_oInput2Decoder.decode( 2, i );
		if ( m_oInput2Decoder.isHeaderReceived( ) )
		{
			// Header received. This happens only once when pressing "play".
			// no useful information in header
		}

		if ( m_oInput2Decoder.isBufferReceived( ) )
		{
			// A buffer has been received, let's check the stimulations inside
			IStimulationSet* l_pStimulations = m_oInput2Decoder.getOutputStimulationSet( );
			for ( uint32 j = 0; j < l_pStimulations->getStimulationCount( ); j++ )
			{
				uint64 l_ui64StimulationCode = l_pStimulations->getStimulationIdentifier( j );
				uint64 l_ui64StimulationDate = l_pStimulations->getStimulationDate( j );
				CString l_sStimulationName = this->getTypeManager( ).getEnumerationEntryNameFromValue( OV_TypeId_Stimulation, l_ui64StimulationCode );
				// If the trigger is received, set the start flag
				if ( l_pStimulations->getStimulationIdentifier( j ) == m_ui64StartTrigger )
				{
					if ( !m_bStartSignalRequested )
					{
						m_bStartSignalRequested = true;
						m_bStopSignalRequested = false; // test purpose, finaly in read back of robo arm
						this->getLogManager( ) << LogLevel_Info << "Starting new try sequence.\n";
					}
					else
					{
						this->getLogManager( ) << LogLevel_Warning << "Start of try sequence received, but sequence is already started!\n";
					}
				}
				else if ( l_pStimulations->getStimulationIdentifier( j ) == m_ui64StopTrigger )
				{
					if ( !m_bStopSignalRequested )
					{
						m_bStopSignalRequested = true;
						m_bStartSignalRequested = false; // test purpose, finaly in read back of robo arm
						this->getLogManager( ) << LogLevel_Info << "Stopoing try sequence.\n";
					}
					else
					{
						this->getLogManager( ) << LogLevel_Warning << "Stop of try sequence received, but sequence is already stopped!\n";
					}
				}
				else
				{
					this->getLogManager() << LogLevel_Warning << "Received an unrecognized trigger, with code [" << l_ui64StimulationCode
						<< "], name [" << l_sStimulationName << "] and date [" << time64(l_ui64StimulationDate) << "]\n";
				}
			}
		}

		if (m_oInput2Decoder.isEndReceived())
		{
			// End received. This happens only once when pressing "stop".
			// nothing to do...
		}
	}
	
	// if threshol is taken from signal
	if ( m_bUseThresholdSignal )
	{
		// now lets process the signal according to current mode
		// iterate over all chunk on input 1 (signal)
		for ( uint32 i = 0; i < l_rDynamicBoxContext.getInputChunkCount( 1 ); i++ )
		{
			// decode the chunk i on input 1
			m_oInput1Decoder.decode( 1, i );
		
			// the decoder may have decoded 3 different parts : the header, a buffer or the end of stream.
			if ( m_oInput1Decoder.isHeaderReceived( ) )
			{
				// Header received. This happens only once when pressing "play".
				// Now we know the sampling rate of the signal, and we can get it:
				// uint64 l_uiSamplingFrequency = m_oInput0Decoder.getOutputSamplingRate();

				IMatrix* l_pMatrix = m_oInput1Decoder.getOutputMatrix( ); // the StreamedMatrix of samples.
				//this->getLogManager() << LogLevel_Info << "CHs:" << std::to_string(static_cast<unsigned long long>(l_pMatrix->getDimensionSize( 0 ))).c_str() << "\n";
				if( l_pMatrix->getDimensionSize( 0 ) != 1 )
				{
					this->getLogManager() << LogLevel_ImportantWarning << "Input Signal matrix does not have 1 dimension - Could not process multidimensional signal!\n";
					return false;
				}
			}

			if ( m_oInput1Decoder.isBufferReceived( ) )
			{
				// Buffer received.
				IMatrix* l_pMatrix = m_oInput1Decoder.getOutputMatrix( ); // the StreamedMatrix of samples.
				uint32 l_ui32ChannelCount = l_pMatrix->getDimensionSize( 0 );
				uint32 l_ui32SamplesPerChannel = l_pMatrix->getDimensionSize( 1 );
				float64* l_pBuffer = l_pMatrix->getBuffer( );
				float64 sum = 0;
	//---------------------------------------------------------------------------------------------------------------------------------------------------------//
				// we can access the sample i from channel j with: l_pBuffer[i+j*l_ui32SamplesPerChannel]
				// in our case we modify every samples, so we only do:
				for ( uint32 j = 0; j < l_pMatrix->getBufferElementCount( ); j++ )
				{
					sum += l_pBuffer[j];
				}
				sum =  sum / l_pMatrix->getBufferElementCount( );
				m_f64ThresholdValue = sum;
	//---------------------------------------------------------------------------------------------------------------------------------------------------------//
			}
	
			if ( m_oInput0Decoder.isEndReceived( ) )
			{
				// End of stream received. This happens only once when pressing "stop". Just pass it to the next boxes so they receive the message :
				// nothig to do ...
			}
		}
	}
	
	// iterate over all chunk on input 0 (signal)
	for ( uint32 i = 0; i < l_rDynamicBoxContext.getInputChunkCount( 0 ); i++ )
	{
		// decode the chunk i on input 0
		m_oInput0Decoder.decode( 0, i );
		
		// the decoder may have decoded 3 different parts : the header, a buffer or the end of stream.
		if ( m_oInput0Decoder.isHeaderReceived( ) )
		{
			// Header received. This happens only once when pressing "play".
			// Now we know the sampling rate of the signal, and we can get it:
			// uint64 l_uiSamplingFrequency = m_oInput0Decoder.getOutputSamplingRate();

			IMatrix* l_pMatrix = m_oInput0Decoder.getOutputMatrix(); // the StreamedMatrix of samples.
			//this->getLogManager() << LogLevel_Info << "CHs:" << std::to_string(static_cast<unsigned long long>(l_pMatrix->getDimensionSize( 0 ))).c_str() << "\n";
			if( l_pMatrix->getDimensionSize( 0 ) != 1 )
			{
				this->getLogManager() << LogLevel_ImportantWarning << "Input Signal matrix does not have 1 dimension - Could not process multidimensional signal!\n";
				return false;
			}
			
			// Pass the header to the next boxes, by encoding a header on the output 0:
			m_oOutput0Encoder.encodeHeader();
			// send the output chunk containing the header. The dates are the same as the input chunk:
			l_rDynamicBoxContext.markOutputAsReadyToSend( 0, l_rDynamicBoxContext.getInputChunkStartTime( 0, i ), l_rDynamicBoxContext.getInputChunkEndTime( 0, i ) );
		}

		if ( m_oInput0Decoder.isBufferReceived( ) )
		{
			// Buffer received.
			IMatrix* l_pMatrix = m_oInput0Decoder.getOutputMatrix( ); // the StreamedMatrix of samples.
			uint32 l_ui32ChannelCount = l_pMatrix->getDimensionSize( 0 );
			uint32 l_ui32SamplesPerChannel = l_pMatrix->getDimensionSize( 1 );
			float64* l_pBuffer = l_pMatrix->getBuffer( );
//---------------------------------------------------------------------------------------------------------------------------------------------------------//
			// according to current mode, we made calculation and compute action
			if ( m_bStartSignalRequested )
			{
				// we can access the sample i from channel j with: l_pBuffer[i+j*l_ui32SamplesPerChannel]
				// in our case we modify every samples, so we only do:
				for ( uint32 j = 0; j < l_pMatrix->getBufferElementCount( ); j++ )
				{
					l_pBuffer[j] = m_f64ThresholdValue;
				}
			}

//---------------------------------------------------------------------------------------------------------------------------------------------------------//
			// Encode the output buffer :
			m_oOutput0Encoder.encodeBuffer();
			// and send it to the next boxes :
			l_rDynamicBoxContext.markOutputAsReadyToSend( 0, l_rDynamicBoxContext.getInputChunkStartTime( 0, i ), l_rDynamicBoxContext.getInputChunkEndTime( 0, i ) );
		}

		if ( m_oInput0Decoder.isEndReceived( ) )
		{
			// End of stream received. This happens only once when pressing "stop". Just pass it to the next boxes so they receive the message :
			m_oOutput0Encoder.encodeEnd();
			l_rDynamicBoxContext.markOutputAsReadyToSend( 0, l_rDynamicBoxContext.getInputChunkStartTime( 0, i ), l_rDynamicBoxContext.getInputChunkEndTime( 0, i ) );
		}

		// The current input chunk has been processed, and automatically discarded thanks to the codec toolkit.
		// you don't need to call "l_rDynamicBoxContext.markInputAsDeprecated(0, i);"
	}

	return true;
}

