// -*- C++ -*-
/*!
 * @file  KobukiMikataArmFingerVisionController.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef KOBUKIMIKATAARMFINGERVISIONCONTROLLER_H
#define KOBUKIMIKATAARMFINGERVISIONCONTROLLER_H

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

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;


// 円柱情報を格納する構造体
struct Cylinder {
    float x;
    float y;
    float z;
    float radius;
    float hue;
    void disp()
    {
        std::cout << "x = " << x << " y = " << y << " z = " << z
            << " radius = " << radius << " hue = " << hue << std::endl;
    }
};

/*!
 * @class KobukiMikataArmFingerVisionController
 * @brief ${rtcParam.description}
 *
 */
class KobukiMikataArmFingerVisionController
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  KobukiMikataArmFingerVisionController(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~KobukiMikataArmFingerVisionController();

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
     int NotFound = 0;

     // 構造体「Cylinder」のベクター
     std::vector<Cylinder>cylinderData; // 構造体「Cylinder」のオブジェクト(インスタンス) (x,y,z,radius,hue)
     // 入力されたm_fv_filter1_wrench.force[1](力の平均z成分 左)の値を保持する
     double FvForceL;
     // 入力されたm_fv_filter0_wrench.force[1](力の平均z成分 右)の値を保持する
     double FvForceR;
     const int StatusBitMoving = 1;
     const int StatusBitExecuting = 2;
     const int StatusBitCommandError = 4;
     const int StatusBitLowerError = 8;
     bool statusMoving;
     bool statusExecuting;
     bool statusCommandError;
     bool statusLowerError;
     double initialFvForceR;        //PreGrasp終了時の右FVの値を保持する
     double initialFvForceL;        //PreGrasp終了時の左FVの値を保持する
     double handRatioGrasp;         // [Grasp]円柱が把持ができた時にhandの閉じ具合を保持する
     double putRatioGrasp;
     int contactCount;              // 接触する条件が満たされる時にこれで判定する
     float FvSlipL[9];
     float FvSlipR[9];
     // 3*3で分けられたオブジェクトの存在を表す配列
     float FvObjExistanceL[9];
     // オブジェクトの中心位置(x,y)
     float FvObjCenterXL;
     float FvObjCenterYL;
     std::vector<double> Planex;
     std::vector<double> Planey;
     std::vector<double> Planez;
};


extern "C"
{
  DLL_EXPORT void KobukiMikataArmFingerVisionControllerInit(RTC::Manager* manager);
};

#endif // KOBUKIMIKATAARMFINGERVISIONCONTROLLER_H
