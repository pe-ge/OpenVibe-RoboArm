
#ifndef __OpenViBEPlugins_Defines_H__
#define __OpenViBEPlugins_Defines_H__

#define OVP_ClassId_BoxAlgorithm_RoboArmStream				OpenViBE::CIdentifier(0x28937175, 0x3E2B37E4)
#define OVP_ClassId_BoxAlgorithm_RoboArmStreamDesc			OpenViBE::CIdentifier(0x695C7E7B, 0x72372EAD)

#define OVP_TypeId_RoboArmStrategy                          OpenViBE::CIdentifier(0x4F7D7BC7, 0x06973AD0)
#define OVP_TypeId_RoboArmStrategy_FullMove                 OpenViBE::CIdentifier(0x42E9650C, 0x66BF52C2)
#define OVP_TypeId_RoboArmStrategy_PartialMoves             OpenViBE::CIdentifier(0x7F121E09, 0x36A028D6)


// @END inserm-gpl

//___________________________________________________________________//
//                                                                   //
// Global defines                                                   //
//___________________________________________________________________//
//                                                                   //

#ifdef TARGET_HAS_ThirdPartyOpenViBEPluginsGlobalDefines
 #include "ovp_global_defines.h"
#endif // TARGET_HAS_ThirdPartyOpenViBEPluginsGlobalDefines


#endif // __OpenViBEPlugins_Defines_H__
