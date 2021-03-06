﻿// -*- C++ -*-
/*!
 * @file  KobukiArmCarry.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef KOBUKIARMCARRY_H
#define KOBUKIARMCARRY_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "BasicDataTypeStub.h"
#include "FingerVisionStub.h"
#include "ExtendedDataTypesStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

/*!
 * @class KobukiArmCarry
 * @brief ${rtcParam.description}
 *
 */
class KobukiArmCarry
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  KobukiArmCarry(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~KobukiArmCarry();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  Arm
   * - DefaultValue: MikataArm
   */
  std::string m_Arm;
  /*!
   * 
   * - Name:  Choreonoid
   * - DefaultValue: 0
   */
  int m_Choreonoid;
  /*!
   * 
   * - Name:  FingerVision
   * - DefaultValue: 0
   */
  int m_FingerVision;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedDoubleSeq m_cylinder;
  /*!
   */
  RTC::InPort<RTC::TimedDoubleSeq> m_cylinderIn;
  RTC::TimedDoubleSeq m_armStatus;
  /*!
   */
  RTC::InPort<RTC::TimedDoubleSeq> m_armStatusIn;
  RTC::TimedString m_inStatus;
  /*!
   */
  RTC::InPort<RTC::TimedString> m_inStatusIn;
  FingerVision::Filter1Wrench m_fv_filter0_wrench;
  /*!
   */
  RTC::InPort<FingerVision::Filter1Wrench> m_fv_filter0_wrenchIn;
  FingerVision::Filter1ObjInfo m_fv_filter0_objinfo;
  /*!
   */
  RTC::InPort<FingerVision::Filter1ObjInfo> m_fv_filter0_objinfoIn;
  FingerVision::Filter1Wrench m_fv_filter1_wrench;
  /*!
   */
  RTC::InPort<FingerVision::Filter1Wrench> m_fv_filter1_wrenchIn;
  FingerVision::Filter1ObjInfo m_fv_filter1_objinfo;
  /*!
   */
  RTC::InPort<FingerVision::Filter1ObjInfo> m_fv_filter1_objinfoIn;
  RTC::TimedDoubleSeq m_plane;
  /*!
   */
  RTC::InPort<RTC::TimedDoubleSeq> m_planeIn;
  RTC::TimedPose2D m_currentPose;
  /*!
   */
  RTC::InPort<RTC::TimedPose2D> m_currentPoseIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_armTipTarget;
  /*!
   */
  RTC::OutPort<RTC::TimedDoubleSeq> m_armTipTargetOut;
  RTC::TimedDoubleSeq m_outData;
  /*!
   */
  RTC::OutPort<RTC::TimedDoubleSeq> m_outDataOut;
  RTC::TimedString m_outStatus;
  /*!
   */
  RTC::OutPort<RTC::TimedString> m_outStatusOut;
  RTC::TimedString m_camera_xy;
  /*!
   */
  RTC::OutPort<RTC::TimedString> m_camera_xyOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void KobukiArmCarryInit(RTC::Manager* manager);
};

#endif // KOBUKIARMCARRY_H
