#ifndef __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__
#define __OpenViBEPlugins_BoxAlgorithm_RoboArmStream_H__

#include <boost/thread.hpp>
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>

#include "ovp_defines.h"
#include "CRoboArmController.h"
#include "CEMSController.h"

namespace OpenViBEPlugins
{
	namespace RoboArm
	{
		/**
		 * \class CBoxAlgorithmRoboArmStream
		 * \author Peter Gergel (UM SAV)
		 * \date Sun Jan 06 10:00:22 2019
		 * \brief The class CBoxAlgorithmRoboArmStream describes the box RoboArm.
		 *
		 */
		class CBoxAlgorithmRoboArmStream : virtual public OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
		{
		public:
            CBoxAlgorithmRoboArmStream(void);
			virtual void release(void) { delete this; }

			virtual OpenViBE::boolean initialize(void);
			virtual OpenViBE::boolean uninitialize(void);
				
			virtual OpenViBE::boolean processInput(uint32_t ui32InputIndex);
			virtual OpenViBE::boolean process(void);
			
			// As we do with any class in openvibe, we use the macro below 
			// to associate this box to an unique identifier. 
			// The inheritance information is also made available, 
			// as we provide the superclass OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
			_IsDerivedFromClass_Final_(OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >, OVP_ClassId_BoxAlgorithm_RoboArmStream);

		protected:
			// Input decoder:
			OpenViBEToolkit::TStimulationDecoder	< CBoxAlgorithmRoboArmStream > m_oInput0Decoder;

			// Store settings values
			OpenViBE::uint32	m_ui32MovementSpeed;
			OpenViBE::uint32	m_ui32MovementSleep;
			OpenViBE::uint32	m_ui32TopAngle;
			OpenViBE::uint32	m_ui32BottomAngle;

			// Robo arm related declarations
			OpenViBE::boolean	m_bRoboArmConnected;
			CRoboArmController	*m_ptRoboArm;
			boost::thread		*m_ptCommunicationHandlerThread;
			OpenViBE::boolean	m_bSimulationRunning;
			OpenViBE::boolean	m_bRecievedStartTrigger;

			// EMS setting
			OpenViBE::boolean	m_bEMSConnected;
			OpenViBE::uint32	m_ui32EMSStimulationTime;
			CEMSController		*m_ptEMS;

			void CBoxAlgorithmRoboArmStream::CommunicationHandler( void );
		};

		/**
		 * \class CBoxAlgorithmovpCBoxAlgorithmRoboArmStreamDesc
		 * \author Peter Gergel (UM SAV)
		 * \date Sun Jan 06 10:00:22 2019
		 * \brief Descriptor of the box RoboArm.
		 *
		 */
		class CBoxAlgorithmRoboArmStreamDesc : virtual public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("RoboArm"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Peter Gergel"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("UM SAV"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Robotic Arm control stream"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("This plugin makes stream of control conmmands to rehabilitation robotic arm."); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("RoboArm"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("1.1"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-properties"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_RoboArmStream; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::RoboArm::CBoxAlgorithmRoboArmStream; }
			

			virtual bool getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rBoxAlgorithmPrototype) const
			{
				rBoxAlgorithmPrototype.addInput("Trigger", OV_TypeId_Stimulations);

				rBoxAlgorithmPrototype.addSetting("Movement speed <1-100>",		OV_TypeId_Integer, "50");
				rBoxAlgorithmPrototype.addSetting("Movement pause (ms)",		OV_TypeId_Integer, "2000");
				rBoxAlgorithmPrototype.addSetting("Top Angle <0-90>",			OV_TypeId_Integer, "60");
				rBoxAlgorithmPrototype.addSetting("Bottom Angle <0-90>",		OV_TypeId_Integer, "60");
				rBoxAlgorithmPrototype.addSetting("EMS stimulation time (ms)",	OV_TypeId_Integer, "1000");
				return true;
			}
			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_RoboArmStreamDesc);
		};
	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_ovpCBoxAlgorithmRoboArmStream_H__
