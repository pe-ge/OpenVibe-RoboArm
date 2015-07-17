#ifndef __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__
#define __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__

#include "ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>

#include <boost/thread.hpp>
#include <windows.h>
#include <stdint.h>
#include "ftd2xx.h"

// The unique identifiers for the box and its descriptor.
// Identifier are randomly chosen by the skeleton-generator.
#define OVP_ClassId_BoxAlgorithm_RoboArmStream OpenViBE::CIdentifier(0x28937175, 0x3E2B37E4)
#define OVP_ClassId_BoxAlgorithm_RoboArmStreamDesc OpenViBE::CIdentifier(0x695C7E7B, 0x72372EAD)

namespace OpenViBEPlugins
{
	namespace RoboArm
	{
		/**
		 * \class CBoxAlgorithmRoboArmStream
		 * \author Peter Gergel, Jakub Bendzala (SAV)
		 * \date Wed Oct 22 13:11:55 2014
		 * \brief The class CBoxAlgorithmRoboArmStream describes the box RoboArmStream.
		 *
		 */
		class CBoxAlgorithmRoboArmStream : virtual public OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
		{

		public:
			CBoxAlgorithmRoboArmStream( void );

			virtual void release(void) { delete this; }

			virtual OpenViBE::boolean initialize ( void );
			virtual OpenViBE::boolean uninitialize ( void );

			//Here is the different process callbacks possible
			// - On clock ticks :
			//virtual OpenViBE::boolean processClock(OpenViBE::CMessageClock& rMessageClock);		
			// - On new input received (the most common behaviour for signal processing) :
			virtual OpenViBE::boolean processInput(OpenViBE::uint32 ui32InputIndex);
			// - On message received :
			virtual OpenViBE::boolean processMessage(const OpenViBE::Kernel::IMessageWithData& msg, OpenViBE::uint32 inputIndex);	
			
			// If you want to use processClock, you must provide the clock frequency.
			//virtual OpenViBE::uint64 getClockFrequency(void);
			
			virtual OpenViBE::boolean process ( void );
			
			// As we do with any class in openvibe, we use the macro below 
			// to associate this box to an unique identifier. 
			// The inheritance information is also made available, 
			// as we provide the superclass OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
			_IsDerivedFromClass_Final_(OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >, OVP_ClassId_BoxAlgorithm_RoboArmStream);


		protected:
			// Input decoder:
			OpenViBEToolkit::TSignalDecoder			< CBoxAlgorithmRoboArmStream > m_oInput0Decoder;
			OpenViBEToolkit::TSignalDecoder			< CBoxAlgorithmRoboArmStream > m_oInput1Decoder;
			OpenViBEToolkit::TStimulationDecoder	< CBoxAlgorithmRoboArmStream > m_oInput2Decoder;
			// Signal stream encoder
			OpenViBEToolkit::TSignalEncoder			< CBoxAlgorithmRoboArmStream > m_oOutput0Encoder;

			// Store settings values
			OpenViBE::CString	m_sStrategy;
			OpenViBE::float64	m_f64ThresholdValue;
			OpenViBE::boolean	m_bUseThresholdSignal;

			OpenViBE::boolean	m_bStartSignalRequested;
			OpenViBE::boolean	m_bStopSignalRequested;
			OpenViBE::uint64	m_ui64StartTrigger;
			OpenViBE::uint64	m_ui64StopTrigger;

			// Robo arm related declarations ----------------------------------------------------------------------------------------------------------------------------------------
			OpenViBE::boolean	m_bFoundRoboticArm;

			static const std::vector<std::string> m_scvMessages;
			static const std::vector<std::string> m_scvCommands;

			HMODULE		m_hmodule;
			FT_HANDLE	m_ftHandle;
			FT_STATUS	ftStatus;
			DWORD		numDevs;
			DWORD		EventDWord;
			DWORD		TxBytes;
			DWORD		RxBytes;
			DWORD		BytesReceived;
			char		RxBuffer[256];
			DWORD		BytesWritten;
			char		TxBuffer[256];

			HANDLE				m_hEvent;
			boost::thread		*m_ptSerialCom;
			OpenViBE::boolean	m_bContinueCommunication;
			OpenViBE::boolean	m_bCommunicationStarted;
			OpenViBE::uint64	m_dReceivedMessage;

			void CBoxAlgorithmRoboArmStream::CommunicationHandler( void );
			
		};

		class CBoxAlgorithmRoboArmStreamDesc : virtual public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("RoboArmStream"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Jakub Bendzala"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("SAV"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Robotic Arm control stream"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("This plugin makes stream of contorl conmmands to rehabilitation robotic arm."); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("RoboArm"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("1.0"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-disconnect"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_RoboArmStream; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::RoboArm::CBoxAlgorithmRoboArmStream; }
			
			virtual OpenViBE::boolean getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rBoxAlgorithmPrototype) const
			{
				rBoxAlgorithmPrototype.addInput("Input Signal",OV_TypeId_Signal);
				rBoxAlgorithmPrototype.addInput("ThresHold Signal",OV_TypeId_Signal);
				rBoxAlgorithmPrototype.addInput("Trigger", OV_TypeId_Stimulations);

				rBoxAlgorithmPrototype.addOutput("Result",OV_TypeId_Signal);

				rBoxAlgorithmPrototype.addSetting("Strategy type", OVP_TypeId_RoboArmStrategy, OVP_TypeId_RoboArmStrategy_FullMove.toString());
				rBoxAlgorithmPrototype.addSetting("Use ThresHold Signal",  OV_TypeId_Boolean, "false");
				rBoxAlgorithmPrototype.addSetting("ThresHold Fixed value",OV_TypeId_Float,"1.0");
				rBoxAlgorithmPrototype.addSetting("Start Trigger", OV_TypeId_Stimulation, "OVTK_StimulationId_SegmentStart");
				rBoxAlgorithmPrototype.addSetting("Stop Trigger", OV_TypeId_Stimulation, "OVTK_StimulationId_SegmentStop");

				rBoxAlgorithmPrototype.addFlag(OpenViBE::Kernel::BoxFlag_IsUnstable);
				
				return true;
			}
			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_RoboArmStreamDesc);
		};

	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__
