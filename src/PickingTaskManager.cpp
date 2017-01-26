// -*- C++ -*-
/*!
 * @file  PickingTaskManager.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "PickingTaskManager.h"
#include "CUIApp.h"

#include <fstream>
#include <sstream>

// Module specification
// <rtc-template block="module_spec">
static const char* pickingtaskmanager_spec[] =
  {
    "implementation_id", "PickingTaskManager",
    "type_name",         "PickingTaskManager",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "ogata-lab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.home_j1", "0",
	"conf.default.home_j2", "-1",
	"conf.default.home_j3", "1",
	"conf.default.home_j4", "0",
	"conf.default.home_j5", "1.5",
	"conf.default.home_j6", "0",
    // Widget
    "conf.__widget__.home_j1", "text",
	"conf.__widget__.home_j2", "text",
	"conf.__widget__.home_j3", "text",
	"conf.__widget__.home_j4", "text",
	"conf.__widget__.home_j5", "text",
	"conf.__widget__.home_j6", "text",
    // Constraints

    "conf.__type__.home_j1", "double",
	"conf.__type__.home_j2", "double",
	"conf.__type__.home_j3", "double",
	"conf.__type__.home_j4", "double",
	"conf.__type__.home_j5", "double",
	"conf.__type__.home_j6", "double",
    ""

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PickingTaskManager::PickingTaskManager(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_ObjectDetectionServicePort("ObjectDetectionService"),
    m_ManipulationPlannerServicePort("ManipulationPlannerService"),
    m_KinematicsSolverServicePort("KinematicsSolverService"),
    m_MotionGeneratorServicePort("MotionGeneratorService"),
    m_manipulatorCommonInterface_MiddlePort("manipulatorCommonInterface_Middle"),
    m_ObjectHandleStrategyServicePort("ObjectHandleStrategyService")
    // </rtc-template>
{

}

/*!
 * @brief destructor
 */
PickingTaskManager::~PickingTaskManager()
{
}



RTC::ReturnCode_t PickingTaskManager::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_ObjectDetectionServicePort.registerConsumer("ObjectDetectionService", "Manipulation::ObjectDetectionService", m_ObjectDetectionService);
  m_ManipulationPlannerServicePort.registerConsumer("ManipulationPlannerService", "Manipulation::ManipulationPlannerService", m_ManipulationPlannerService);
  m_KinematicsSolverServicePort.registerConsumer("Manipulation_KinematicSolverService", "Manipulation::KinematicSolverService", m_KinematicsSolverService);
  m_MotionGeneratorServicePort.registerConsumer("MotionGeneratorService", "Manipulation::MotionGeneratorService", m_MotionGeneratorService);
  m_manipulatorCommonInterface_MiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_manipulatorCommonInterface_Middle);
  m_ObjectHandleStrategyServicePort.registerConsumer("ObjectHandleStrategyService", "Manipulation::ObjectHandleStrategyService", m_ObjectHandleStrategyService);
  
  // Set CORBA Service Ports
  addPort(m_ObjectDetectionServicePort);
  addPort(m_ManipulationPlannerServicePort);
  addPort(m_KinematicsSolverServicePort);
  addPort(m_MotionGeneratorServicePort);
  addPort(m_manipulatorCommonInterface_MiddlePort);
  addPort(m_ObjectHandleStrategyServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  bindParameter("home_j1", m_home_j1, "0");
  bindParameter("home_j2", m_home_j2, "-1");
  bindParameter("home_j3", m_home_j3, "1");
  bindParameter("home_j4", m_home_j4, "0");
  bindParameter("home_j5", m_home_j5, "1.5");
  bindParameter("home_j6", m_home_j6, "0");

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PickingTaskManager::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PickingTaskManager::onActivated(RTC::UniqueId ec_id)
{
  m_app = new CUIApp(this);
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PickingTaskManager::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PickingTaskManager::onExecute(RTC::UniqueId ec_id)
{
  std::cout << "Please input command (h:help):" << std::endl;
  char c;
  std::cin >>c;

  switch(c){

  case '0':
	  m_home_j[0]=m_home_j1;
	  m_home_j[1]=m_home_j2;
	  m_home_j[2]=m_home_j3;
	  m_home_j[3]=m_home_j4;
	  m_home_j[4]=m_home_j5;
	  m_home_j[5]=m_home_j6;
	  m_app->goStartPosition(m_home_j);
	  break;

  case '1':
	  m_app->detectObj();
	  break;

  case '2':
	  m_app->determineApproachPose();
	  break;

  case '3':
	  m_app->solveKinematics();
	  break;

  case '4':
	  m_app->searchMotionPlan();
	  //m_app->searchReplacingPlan();
	  break;

  case '5':
	  m_app->generateMotionPlan();
	  break;

  case 's':
          m_app->showParams();
    	  
  case 'h':
	  std::cout << "0: move to start pose" << std::endl;
	  std::cout << "1: detect target object" << std::endl;
	  std::cout << "2: determine end effector's pose" << std::endl;
	  std::cout << "3: solce inverse kinematics" << std::endl;
	  std::cout << "4: search path" << std::endl;
	  std::cout << "5: follow path" << std::endl;
	  std::cout << "s: show current parameters" <<std::endl;
	  std::cout << "h: help" << std::endl;
	  //std::cout << "c: generate motion form csv" << std::endl;
	  break;

  //case 'c':
	  //m_app->setSampleManipPlan();
	  //break;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PickingTaskManager::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PickingTaskManager::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

Manipulation::ReturnValue* PickingTaskManager::callDetectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo){
	return m_ObjectDetectionService->detectObject(objectID, objInfo);
}

Manipulation::ReturnValue* PickingTaskManager::callSolveKinematics(const Manipulation::EndEffectorPose& targetPose, Manipulation::JointAngleSeq startJointAngles,
																   Manipulation::JointAngleSeq_out targetJointAngles){
	return m_KinematicsSolverService->solveKinematics(targetPose, startJointAngles, targetJointAngles);
}

Manipulation::ReturnValue* PickingTaskManager::callGetCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
	return m_MotionGeneratorService->getCurrentRobotJointAngles(jointAngles);
}

Manipulation::ReturnValue* PickingTaskManager::callPlanManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::JointAngleSeq& startJointAngles,
																	const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan){
	return m_ManipulationPlannerService->planManipulation(robotID, startJointAngles, goalJointAngles, manipPlan);
}

Manipulation::ReturnValue* PickingTaskManager::callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan){
	return m_MotionGeneratorService->followManipPlan(manipPlan);
}

Manipulation::ReturnValue* PickingTaskManager::callGetApproachOrientation(const Manipulation::ObjectInfo& objInfo, Manipulation::EndEffectorPose_out eePos){
	return m_ObjectHandleStrategyService->getApproachOrientation(objInfo, eePos);
}

void PickingTaskManager::callMoveGripper(const int degree){
        m_manipulatorCommonInterface_Middle->moveGripper(degree);
}

void PickingTaskManager::callOpenGripper(){
        m_manipulatorCommonInterface_Middle->openGripper();
}

void PickingTaskManager::callSetHome(const JARA_ARM::JointPos_var jpos){
	m_manipulatorCommonInterface_Middle->setHome(jpos);
}

void PickingTaskManager::callGoHome(){
	m_manipulatorCommonInterface_Middle->goHome();
}

void PickingTaskManager::callSetSpeedJoint(unsigned long spdRation){
	m_manipulatorCommonInterface_Middle->setSpeedJoint(spdRation);
}

void PickingTaskManager::callMovePTPJointAbs(const JARA_ARM::JointPos_var jpos){
	m_manipulatorCommonInterface_Middle->movePTPJointAbs(jpos);

}


extern "C"
{
 
  void PickingTaskManagerInit(RTC::Manager* manager)
  {
    coil::Properties profile(pickingtaskmanager_spec);
    manager->registerFactory(profile,
                             RTC::Create<PickingTaskManager>,
                             RTC::Delete<PickingTaskManager>);
  }
  
};
