
#include <vector>
#include <openvibe/ov_all.h>
#include "ovp_defines.h"

#include "ovpCBoxAlgorithmRoboArmStream.h"


OVP_Declare_Begin();

	OVP_Declare_New(OpenViBEPlugins::RoboArm::CBoxAlgorithmRoboArmStreamDesc);

	rPluginModuleContext.getTypeManager().registerEnumerationType (OVP_TypeId_RoboArmStrategy, "Control Strategy");
	rPluginModuleContext.getTypeManager().registerEnumerationEntry(OVP_TypeId_RoboArmStrategy, "Full Move", OVP_TypeId_RoboArmStrategy_FullMove.toUInteger());
	rPluginModuleContext.getTypeManager().registerEnumerationEntry(OVP_TypeId_RoboArmStrategy, "Partial Moves", OVP_TypeId_RoboArmStrategy_PartialMoves.toUInteger());

		 
OVP_Declare_End();
