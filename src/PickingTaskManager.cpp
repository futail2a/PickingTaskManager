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
    m_MotionGeneratorServicePort("MotionGeneratorService")

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
  m_KinematicsSolverServicePort.registerConsumer("KinematicsSolverService", "Manipulation::KinematicsSolverService", m_KinematicsSolverService);
  m_MotionGeneratorServicePort.registerConsumer("MotionGeneratorService", "Manipulation::MotionGeneratorService", m_MotionGeneratorService);
  
  // Set CORBA Service Ports
  addPort(m_ObjectDetectionServicePort);
  addPort(m_ManipulationPlannerServicePort);
  addPort(m_KinematicsSolverServicePort);
  addPort(m_MotionGeneratorServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
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
  
  case 'd':
	  m_app->detectObj();
	  break;

  case 'p':
	  m_app->searchMotionPlan();
	  break;

  case 'g':
	  m_app->generateMotionPlan();
	  break;

  case 'e':
	  std::cout << "Generate motion from a csv file" << std::endl;
	  m_app->setSampleManipPlan();
	  //std::cout << "Test solving kinematics.." << std::endl;
	  //m_KinematicsSolverService->solveInverseKinematics((*m_objInfo), m_goalRobotJointInfo);
	  break;

  case's':
	  m_app->showParams();
	  break;
	  
  case 'h':
	  std::cout << "d: detect target object" << std::endl;
	  std::cout << "p: search motion plan" << std::endl;
	  std::cout << "g: generate motion plan" << std::endl;
	  std::cout << "s: show current parameters" << std::endl;

	  std::cout << "h: help" << std::endl;
	  std::cout << "e: debug mode" << std::endl;
	  break;
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

void PickingTaskManager::callDetectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo){
	m_ObjectDetectionService->detectObject(objectID, objInfo);
}

void PickingTaskManager::callSolveInverseKinematics(const Manipulation::ObjectInfo& objInfo, Manipulation::RobotJointInfo_out goalRobotJointInfo){
	m_KinematicsSolverService->solveInverseKinematics(objInfo, goalRobotJointInfo);
}

void PickingTaskManager::callGetCurrentRobotJointInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJoint){
	m_MotionGeneratorService->getCurrentRobotJointInfo(robotID, robotJoint);
}

void PickingTaskManager::callPlanManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan){
	m_ManipulationPlannerService->planManipulation(robotID, startRobotJointInfo, goalRobotJointInfo, manipPlan);
}

void PickingTaskManager::callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan){
	m_MotionGeneratorService->followManipPlan(manipPlan);
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
