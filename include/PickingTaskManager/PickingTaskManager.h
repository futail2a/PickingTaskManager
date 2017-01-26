// -*- C++ -*-
/*!
 * @file  PickingTaskManager.h
 * @brief ModuleDescription
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PICKINGTASKMANAGER_H
#define PICKINGTASKMANAGER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "TrajectoryPlannerStub.h"
#include "ManipulatorCommonInterface_MiddleLevelStub.h"
#include "MotionGeneratorServiceDecorator.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

class CUIApp;
class MotionGeneratorServiceDecorator;

/*!
 * @class PickingTaskManager
 * @brief ModuleDescription
 *
 */
class PickingTaskManager
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  PickingTaskManager(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~PickingTaskManager();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
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
   * former rtc_stopping_entry()
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
   * former rtc_active_entry()
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
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
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
   * former rtc_aborting_entry()
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
   * former rtc_error_do()
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
   * This is same but different the former rtc_init_entry()
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
   * no corresponding operation exists in OpenRTm-aist-0.2.0
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
   * no corresponding operation exists in OpenRTm-aist-0.2.0
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
   double m_home_j1;
   double m_home_j2;
   double m_home_j3;
   double m_home_j4;
   double m_home_j5;
   double m_home_j6;
   double m_home_j[6];
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_ObjectDetectionServicePort;
  /*!
   */
  RTC::CorbaPort m_ManipulationPlannerServicePort;
  /*!
   */
  RTC::CorbaPort m_KinematicsSolverServicePort;
  /*!
   */
  RTC::CorbaPort m_MotionGeneratorServicePort;
  /*!
   */
  RTC::CorbaPort m_manipulatorCommonInterface_MiddlePort;
  /*!
   */
  RTC::CorbaPort m_ObjectHandleStrategyServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<Manipulation::ObjectDetectionService> m_ObjectDetectionService;
  /*!
   */
  RTC::CorbaConsumer<Manipulation::ManipulationPlannerService> m_ManipulationPlannerService;
  /*!
   */
  RTC::CorbaConsumer<Manipulation::KinematicSolverService> m_KinematicsSolverService;
  /*!
   */
  RTC::CorbaConsumer<Manipulation::MotionGeneratorService> m_MotionGeneratorService;
  MotionGeneratorServiceDecorator* m_MotionGeneratorServiceDecorator;
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Middle> m_manipulatorCommonInterface_Middle;
  /*!
   */
  RTC::CorbaConsumer<Manipulation::ObjectHandleStrategyService> m_ObjectHandleStrategyService;
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>]

  CUIApp* m_app;

class DisconnCallback: public RTC::ConnectionCallback{
private:
  PickingTaskManager* m_rtc;
  
public:
  DisconnCallback(PickingTaskManager* ptr){m_rtc=ptr;}
  //~DisconnCallback(){std::cout<<"Disconnection Callback"<<std::endl;}

  void operator()(RTC::ConnectorProfile& profile);
};

class ConnCallback: public RTC::ConnectionCallback{
private:
  PickingTaskManager* m_rtc;
  
public:
  ConnCallback(PickingTaskManager* ptr){m_rtc=ptr;}
  //~ConnCallback(){std::cout<<"Connection Callback"<<std::endl;}
  
  void operator()(RTC::ConnectorProfile& profile);
};
	 double home_j[6]={};

 
 public:
	 Manipulation::ReturnValue* callDetectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo);
	 Manipulation::ReturnValue* callSolveKinematics(const Manipulation::EndEffectorPose& targetPose, Manipulation::JointAngleSeq startJointAngles,
			 	 	 	 	 	 	 	 	 	 Manipulation::JointAngleSeq_out targetJointAngles);
	 Manipulation::ReturnValue* callGetCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles);
	 Manipulation::ReturnValue* callPlanManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::JointAngleSeq& startJointAngles,
			 	 	 	 	 	 	 	 	 	 	 const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan);
	 Manipulation::ReturnValue* callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan);
	 Manipulation::ReturnValue* callGetApproachOrientation(const Manipulation::ObjectInfo& objInfo, Manipulation::EndEffectorPose_out eePos);

	 void refreshManipPlan(const Manipulation::ManipulationPlan& manipPlan, Manipulation::ManipulationPlan_out newPlan);

void callMoveGripper(const int degree);
void callOpenGripper();
void callSetHome(const JARA_ARM::JointPos_var jpos);
void callGoHome();
void callMovePTPJointAbs(const JARA_ARM::JointPos_var jpos);
void callSetSpeedJoint(unsigned long spdRation);
void callMovePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carpos);
  
};

extern "C"
{
  DLL_EXPORT void PickingTaskManagerInit(RTC::Manager* manager);
  };

#endif // PICKINGTASKMANAGER_H
