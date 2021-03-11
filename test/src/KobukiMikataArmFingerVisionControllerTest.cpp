// -*- C++ -*-
/*!
 * @file  KobukiMikataArmFingerVisionControllerTest.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "KobukiMikataArmFingerVisionControllerTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* kobukimikataarmfingervisioncontroller_spec[] =
  {
    "implementation_id", "KobukiMikataArmFingerVisionControllerTest",
    "type_name",         "KobukiMikataArmFingerVisionControllerTest",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Test",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
KobukiMikataArmFingerVisionControllerTest::KobukiMikataArmFingerVisionControllerTest(RTC::Manager* manager)
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
KobukiMikataArmFingerVisionControllerTest::~KobukiMikataArmFingerVisionControllerTest()
{
}



RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onInitialize()
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
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiMikataArmFingerVisionControllerTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void KobukiMikataArmFingerVisionControllerTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(kobukimikataarmfingervisioncontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<KobukiMikataArmFingerVisionControllerTest>,
                             RTC::Delete<KobukiMikataArmFingerVisionControllerTest>);
  }
  
};


