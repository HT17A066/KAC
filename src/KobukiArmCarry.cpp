// -*- C++ -*-
/*!
 * @file  KobukiArmCarry.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "KobukiArmCarry.h"

// Module specification
// <rtc-template block="module_spec">
static const char* kobukiarmcarry_spec[] =
  {
    "implementation_id", "KobukiArmCarry",
    "type_name",         "KobukiArmCarry",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Test",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.Arm", "MikataArm",
    "conf.default.Choreonoid", "0",
    "conf.default.FingerVision", "0",

    // Widget
    "conf.__widget__.Arm", "radio",
    "conf.__widget__.Choreonoid", "radio",
    "conf.__widget__.FingerVision", "radio",
    // Constraints
    "conf.__constraints__.Arm", "(MikataArm,Crane+)",
    "conf.__constraints__.Choreonoid", "(0,1)",
    "conf.__constraints__.FingerVision", "(0,1)",

    "conf.__type__.Arm", "string",
    "conf.__type__.Choreonoid", "int",
    "conf.__type__.FingerVision", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
KobukiArmCarry::KobukiArmCarry(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_cylinderIn("cylinder", m_cylinder),
    m_armStatusIn("armStatus", m_armStatus),
    m_inStatusIn("inStatus", m_inStatus),
    m_fv_filter0_wrenchIn("fv_filter0_wrench", m_fv_filter0_wrench),
    m_fv_filter0_objinfoIn("fv_filter0_objinfo", m_fv_filter0_objinfo),
    m_fv_filter1_wrenchIn("fv_filter1_wrench", m_fv_filter1_wrench),
    m_fv_filter1_objinfoIn("fv_filter1_objinfo", m_fv_filter1_objinfo),
    m_planeIn("plane", m_plane),
    m_currentPoseIn("currentPose", m_currentPose),
    m_armTipTargetOut("armTipTarget", m_armTipTarget),
    m_outDataOut("outData", m_outData),
    m_outStatusOut("outStatus", m_outStatus),
    m_camera_xyOut("camera_xy", m_camera_xy)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
KobukiArmCarry::~KobukiArmCarry()
{
}



RTC::ReturnCode_t KobukiArmCarry::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("cylinder", m_cylinderIn);
  addInPort("armStatus", m_armStatusIn);
  addInPort("inStatus", m_inStatusIn);
  addInPort("fv_filter0_wrench", m_fv_filter0_wrenchIn);
  addInPort("fv_filter0_objinfo", m_fv_filter0_objinfoIn);
  addInPort("fv_filter1_wrench", m_fv_filter1_wrenchIn);
  addInPort("fv_filter1_objinfo", m_fv_filter1_objinfoIn);
  addInPort("plane", m_planeIn);
  addInPort("currentPose", m_currentPoseIn);
  
  // Set OutPort buffer
  addOutPort("armTipTarget", m_armTipTargetOut);
  addOutPort("outData", m_outDataOut);
  addOutPort("outStatus", m_outStatusOut);
  addOutPort("camera_xy", m_camera_xyOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Arm", m_Arm, "MikataArm");
  bindParameter("Choreonoid", m_Choreonoid, "0");
  bindParameter("FingerVision", m_FingerVision, "0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiArmCarry::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t KobukiArmCarry::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiArmCarry::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiArmCarry::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiArmCarry::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarry::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void KobukiArmCarryInit(RTC::Manager* manager)
  {
    coil::Properties profile(kobukiarmcarry_spec);
    manager->registerFactory(profile,
                             RTC::Create<KobukiArmCarry>,
                             RTC::Delete<KobukiArmCarry>);
  }
  
};


