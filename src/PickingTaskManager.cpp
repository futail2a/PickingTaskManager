// -*- C++ -*-
/*!
 * @file  PickingTaskManager.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "PickingTaskManager.h"
#include <fstream>
#include<sstream>
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
void PickingTaskManager::initParams()
{
	m_objectID = new Manipulation::ObjectIdentifier();
	m_objInfo = new Manipulation::ObjectInfo();

	m_robotID = new Manipulation::RobotIdentifier();
	m_robotJoint = new Manipulation::RobotJointInfo();

	m_startRobotJointInfo = new Manipulation::RobotJointInfo();
	m_goalRobotJointInfo = new Manipulation::RobotJointInfo();
	m_manipPlan = new Manipulation::ManipulationPlan();
}

RTC::ReturnCode_t PickingTaskManager::onActivated(RTC::UniqueId ec_id)
{
  initParams();
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
  std::cout << "Please input command:" << std::endl;
  char c;
  std::cin >>c;

  switch(c){
  /*
  case 's':
	std::cout << "Start" << std::endl;
	break;
  */
  case 'd':
	  std::cout << "Test detecting objects.." << std::endl;
	  m_ObjectDetectionService->detectObject((*m_objectID), m_objInfo);
	  break;

  case 'k':
	  std::cout << "Test solving kinematics.." << std::endl;
	  m_KinematicsSolverService->solveInverseKinematics((*m_objInfo), m_goalRobotJointInfo);
	  break;

  case 'p':
	  std::cout << "Test plannig.." << std::endl;

	  m_MotionGeneratorService->getCurrentRobotJointInfo((*m_robotID), m_robotJoint);
	  m_ManipulationPlannerService->planManipulation((*m_robotID), (*m_startRobotJointInfo), (*m_goalRobotJointInfo), m_manipPlan);
	  break;

  case 'g':
	  std::cout << "Test motion generation.." << std::endl;
	  m_manipPlan;
	  m_MotionGeneratorService->followManipPlan((*m_manipPlan));
	  break;

  case't':
	  setSampleManipPlan();
	  for(int i =0;i<m_manipPlan->robotJointInfoSeq.length(); i++){
		  for(int j=0;j<6;j++){
			  std::cout << m_manipPlan->robotJointInfoSeq[i].jointInfoSeq[j].jointAngle << " ";
		  }
		  std::cout <<std::endl;
	  }
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



//for debugging only, delete this function before release
void PickingTaskManager::setSampleManipPlan(){
    std::string str;
	std::ifstream ifs("sampleManipPath.csv");

    if (!ifs){
        std::cout << "error" << std::endl;
    }

    while(getline(ifs,str))
    {
        std::string tmp;
        std::istringstream stream(str);
    	Manipulation::RobotJointInfo posture;

        while(getline(stream,tmp,','))
        {
        	Manipulation::JointInfo joint;
        	joint.jointAngle = std::stod(tmp);

        	CORBA::ULong len = posture.jointInfoSeq.length();
            posture.jointInfoSeq.length(len + 1);
            posture.jointInfoSeq[len] = joint;
    	}

        CORBA::ULong len = m_manipPlan->robotJointInfoSeq.length();
        m_manipPlan->robotJointInfoSeq.length(len + 1);
	    m_manipPlan->robotJointInfoSeq[len]=posture;
    }

}
