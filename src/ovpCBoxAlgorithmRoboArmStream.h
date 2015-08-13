#ifndef __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__
#define __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__

#include "ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>
#include "CRoboArmController.h"

#include <boost/thread.hpp>
#include <windows.h>
#include <stdint.h>

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
		 * \author Peter Gergel
		 * \date Wed Oct 22 13:11:55 2014
		 * \brief The class CBoxAlgorithmRoboArmStream describes the box RoboArmStream.
		 *
		 */
		class CBoxAlgorithmRoboArmStream : virtual public OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
		{

		public:
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
			OpenViBEToolkit::TStimulationDecoder	< CBoxAlgorithmRoboArmStream > m_oInput0Decoder;

			// Store settings values
			OpenViBE::uint64	m_ui64TopAngle;
			OpenViBE::uint64	m_ui64BottomAngle;

			// Robo arm related declarations
			CRoboArmController	*m_ptRoboArm;
			boost::thread		*m_ptCommunicationHandlerThread;
			OpenViBE::boolean	m_bSimulationRunning;

			void CBoxAlgorithmRoboArmStream::CommunicationHandler( void );
		};

		class CBoxAlgorithmRoboArmStreamDesc : virtual public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("RoboArmStream"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Peter Gergel"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("SAV"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Robotic Arm control stream"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("This plugin makes stream of control conmmands to rehabilitation robotic arm."); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("RoboArm"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("1.0"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-disconnect"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_RoboArmStream; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::RoboArm::CBoxAlgorithmRoboArmStream; }
			
			virtual OpenViBE::boolean getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rBoxAlgorithmPrototype) const
			{
				rBoxAlgorithmPrototype.addInput("Trigger", OV_TypeId_Stimulations);
				rBoxAlgorithmPrototype.addSetting("Top Angle",				OV_TypeId_Integer,			"90");
				rBoxAlgorithmPrototype.addSetting("Bottom Angle",			OV_TypeId_Integer,			"90");
				return true;
			}
			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_RoboArmStreamDesc);
		};

	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__
