#include "ovpCBoxAlgorithmRoboArmStream.h"

#include <string>
#include <iostream>

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
#include "ftd2xx.h"

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::RoboArm;

using namespace boost;
using namespace boost::this_thread;

const char *m_scvMessages_init[] = {"HW::ok", "START::ok", "STOP::ok", "STEP::ok", "ANGLE::ok"};
const std::vector<std::string> CBoxAlgorithmRoboArmStream::m_scvMessages( m_scvMessages_init, std::end( m_scvMessages_init ) ); // definition of messgaes

const char *m_scvCommands_init[] = {"HW?", "START=", "STOP", "STEP=", "SET+ANGLE="};
const std::vector<std::string> CBoxAlgorithmRoboArmStream::m_scvCommands( m_scvCommands_init, std::end( m_scvCommands_init ) ); // definition of messgaes

CBoxAlgorithmRoboArmStream::CBoxAlgorithmRoboArmStream( void ) :
	m_ptSerialCom(NULL)
{
};

void CBoxAlgorithmRoboArmStream::CommunicationHandler( void )
{
	char input_buffer[32];
	unsigned char recieved_chars = 0;
	FT_STATUS status;

	while ( m_bContinueCommunication )
	{
		DWORD dwRead, dwRXBytes;
		unsigned char b;

		m_bCommunicationStarted = true;

		// wait for event - RX char
		WaitForSingleObject( m_hEvent, -1 );
		if ( m_ftHandle )
		{
			status = FT_GetQueueStatus( m_ftHandle, &dwRead );
			if ( status != FT_OK )
			{	// error check
//				MessageBox::Show("GError");
				continue;
			}
			// read characters while there are any
			while (dwRead && m_bContinueCommunication )
			{
				// get one character
				status = FT_Read( m_ftHandle, &b, 1, &dwRXBytes );
				if ( status != FT_OK )
				{	// error check
//					MessageBox::Show("RError");
					continue;
				}
				else
				{
					// do something with received character 'b'
					if ( b != '\r' && recieved_chars < 32 )
					{
						input_buffer[recieved_chars++] = b;
					}
					else // end of message
					{
						// chcek if buffer overflowed
						if ( recieved_chars >=32 )
						{
							recieved_chars = 31; // hendle buffer overflow
						}
						input_buffer[recieved_chars] = NULL; // terminate message string
						std::string str_intpu_buffer( input_buffer );
						for ( unsigned char ms = 0; ms < m_scvMessages.size(); ms++ )
						{
							// check if there is some message from the list in the buffer
							if ( str_intpu_buffer.find( m_scvMessages[ms] ) != std::string::npos )
							{
								m_dReceivedMessage = ms + 1;
							}
						}
						recieved_chars = 0;
						input_buffer[0] = NULL;
					}
				}
				status = FT_GetQueueStatus( m_ftHandle, &dwRead );
			} //end of loop
		}
		this_thread::sleep( posix_time::milliseconds( 10 ) );
	}

	if ( m_ftHandle )
	{
		FT_Close(m_ftHandle);
		try
		{
			// Sleep and check for interrupt. 
			// To check for interrupt without sleep, use boost::this_thread::interruption_point()
			// which also throws boost::thread_interrupted
			this_thread::sleep( posix_time::milliseconds( 50 ) );
		}
		catch ( boost::thread_interrupted& )
		{
			m_bCommunicationStarted = false;
			return;
		}
	}
}



boolean CBoxAlgorithmRoboArmStream::initialize ( void )
{
	CIdentifier l_oInputTypeIdentifier;

	getStaticBoxContext( ).getInputType( 0, l_oInputTypeIdentifier );
	if ( l_oInputTypeIdentifier != OV_TypeId_Signal )
	{
		this->getLogManager() << LogLevel_Error << 
			"Invalid input type of Intpu signal!" << 
			std::to_string( static_cast<unsigned long long > ( ftStatus ) ).c_str( ) << "\n";
		return false;
	}

	getStaticBoxContext( ).getInputType( 1, l_oInputTypeIdentifier );
	if ( l_oInputTypeIdentifier != OV_TypeId_Signal )
	{
		this->getLogManager() << LogLevel_Error << 
			"Invalid input type of ThresHold signal!" << 
			std::to_string( static_cast<unsigned long long > ( ftStatus ) ).c_str( ) << "\n";
		return false;
	}

	getStaticBoxContext( ).getInputType( 2, l_oInputTypeIdentifier );
	if ( l_oInputTypeIdentifier != OV_TypeId_Stimulations )
	{
		this->getLogManager() << LogLevel_Error << 
			"Invalid input type of Stimulation input!" << 
			std::to_string( static_cast<unsigned long long > ( ftStatus ) ).c_str( ) << "\n";
		return false;
	}

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
	// Get Manual Threshol value
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


	//FT_STATUS ftStatus;
	//DWORD numDevs;
	m_bFoundRoboticArm = false;

	FT_Close(m_ftHandle); // must be closed to perform the ListDevices( ) function
	//long m_NumRecd = 0;

	ftStatus = FT_ListDevices( &numDevs, NULL, FT_LIST_NUMBER_ONLY );
	if ( ftStatus == FT_OK )
	{
		// FT_ListDevices OK, show number of devices connected in list box
		this->getLogManager( ) << LogLevel_Info << "FTDI number of devices: " << std::to_string( static_cast<unsigned long long>( numDevs ) ).c_str( ) << "\n";

		// list descriptions of all connected devices
		if ( numDevs > 0 )
		{
			ftStatus = FT_ListDevices( &numDevs, NULL, FT_LIST_NUMBER_ONLY );
			if ( ftStatus == FT_OK ) 
			{
				char *BufPtrs[64]; // pointer to array of 64 pointers
				DWORD d;
				for ( d = 0; d < numDevs; d++ )
				{
					BufPtrs[d] = new char[64];
				}
				BufPtrs[d] = NULL;

				ftStatus = FT_ListDevices( BufPtrs, &numDevs, FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION );
				if ( FT_SUCCESS( ftStatus ) ) 
				{
					for ( DWORD u = 0; u < numDevs; u++ )
					{
						this->getLogManager( ) << LogLevel_Info << "FTDI found device: " << std::string( BufPtrs[u] ).c_str() << "\n";
						if ( std::string( BufPtrs[u] ).find( "TTL232R-3V3" ) )
						{
							m_bFoundRoboticArm = true;
						}
					}
				}
				else 
				{
					this->getLogManager( ) << LogLevel_Error << "ListDevices failed!\n";	
				}

				//free ram to avoid memory leaks
				for ( d = 0; d < numDevs; d++ )
				{
					delete BufPtrs[d];
				}

				// check if robotic arm is connected
				if ( !m_bFoundRoboticArm )
				{
					this->getLogManager( ) << LogLevel_Error << "Robotic arm is not connected!\n";
					return false;
				}
			}		
		}
		else
		{
			// no devices connected
			this->getLogManager( ) << LogLevel_Error << "Robotic arm is not connected!\n";
			return false;
		}
	}
	else 
	{
		// FT_ListDevices failed
		this->getLogManager( ) << LogLevel_Error << "FT_ListDevices failed!\n";
		return false;
	}

	//ftStatus = OpenEx((PVOID)(LPCTSTR)"FT232R USB UART", FT_OPEN_BY_DESCRIPTION );
	ftStatus = FT_OpenEx((PVOID)(LPCTSTR)"FT232R USB UART", FT_OPEN_BY_DESCRIPTION, &m_ftHandle);

	char txbuf[32], rxbuf[32];
	DWORD ret_bytes = 0;

	/*this->ResetDevice( );
	this->Purge( FT_PURGE_RX || FT_PURGE_TX );
	this->ResetDevice( );
	this->SetTimeouts( 3000, 3000 ); //extend timeout while board finishes reset
	Sleep( 150 );*/

	// test for presence of RoboticArm
	sprintf( txbuf, "HW?\r" );
	//sprintf( txbuf, "START=30\r" );
	ftStatus = FT_Write(m_ftHandle, txbuf, std::string( txbuf ).length() , &ret_bytes);
	ftStatus = FT_Read(m_ftHandle, rxbuf, 1, &ret_bytes);
	if ( ret_bytes == 0 )
	{
		ftStatus = FT_Read(m_ftHandle, rxbuf, 6, &ret_bytes);
	}
	rxbuf[ret_bytes] = NULL;

	if ( !std::strstr( rxbuf, "HW::ok" ) )
	{
		this->getLogManager( ) << LogLevel_Error << "Robotic arm is not responding!\n";
		FT_Close(m_ftHandle);
	}	
	else
	{
		this->getLogManager( ) << LogLevel_Info << "RoboticArm response: " << std::string( rxbuf ).c_str( ) << "\n";
		//this->getLogManager( ) << LogLevel_Info << "RoboticArm is ready!\n";
	}


	// Strat reading thread ----------------------------------------------------------------------------------------------------------------------------------------
	m_bContinueCommunication = true;
	m_bCommunicationStarted = false;
	m_dReceivedMessage = 0;
	m_ptSerialCom = new boost::thread( boost::bind( &CBoxAlgorithmRoboArmStream::CommunicationHandler, this ) );

	ftStatus = FT_SetEventNotification( m_ftHandle, FT_EVENT_RXCHAR, m_hEvent );
	ftStatus = FT_SetBaudRate( m_ftHandle, 57600 );

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
	if ( m_hEvent ) // let the thread come out of waiting for infinite time
	{
		SetEvent( m_hEvent );
	}
	
	if (m_ptSerialCom != NULL)
	{
		m_ptSerialCom->interrupt( ); // Ask thread to stop
	}
	//m_ptSerialCom->join( ); // Join - wait when thread actually exits

	FT_Close(m_ftHandle);

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

	if ( m_dReceivedMessage )
	{
		this->getLogManager( ) << LogLevel_Info << "Receieved message: " << std::to_string( static_cast<unsigned long long > ( m_dReceivedMessage ) ).c_str( ) << "\n";
		m_dReceivedMessage = 0;
	}
	
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

