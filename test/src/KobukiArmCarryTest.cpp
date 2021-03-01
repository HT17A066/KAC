// -*- C++ -*-
/*!
 * @file  KobukiArmCarryTest.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "KobukiArmCarryTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* kobukiarmcarry_spec[] =
  {
    "implementation_id", "KobukiArmCarryTest",
    "type_name",         "KobukiArmCarryTest",
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
KobukiArmCarryTest::KobukiArmCarryTest(RTC::Manager* manager)
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
KobukiArmCarryTest::~KobukiArmCarryTest()
{
}



RTC::ReturnCode_t KobukiArmCarryTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("armTipTarget", m_armTipTargetIn);
  addInPort("outData", m_outDataIn);
  addInPort("outStatus", m_outStatusIn);
  addInPort("camera_xy", m_camera_xyIn);
  
  // Set OutPort buffer
  addOutPort("cylinder", m_cylinderOut);
  addOutPort("armStatus", m_armStatusOut);
  addOutPort("inStatus", m_inStatusOut);
  addOutPort("fv_filter0_wrench", m_fv_filter0_wrenchOut);
  addOutPort("fv_filter0_objinfo", m_fv_filter0_objinfoOut);
  addOutPort("fv_filter1_wrench", m_fv_filter1_wrenchOut);
  addOutPort("fv_filter1_objinfo", m_fv_filter1_objinfoOut);
  addOutPort("plane", m_planeOut);
  addOutPort("currentPose", m_currentPoseOut);
  
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
RTC::ReturnCode_t KobukiArmCarryTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t KobukiArmCarryTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiArmCarryTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiArmCarryTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiArmCarryTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiArmCarryTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void KobukiArmCarryTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(kobukiarmcarry_spec);
    manager->registerFactory(profile,
                             RTC::Create<KobukiArmCarryTest>,
                             RTC::Delete<KobukiArmCarryTest>);
  }
  
};


