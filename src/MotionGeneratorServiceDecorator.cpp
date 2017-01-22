// -*-C++-*-
#include "MotionGeneratorServiceDecorator.h"
#include <thread>
#include <exception>

MotionGeneratorServiceDecorator::MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pCorbaConsumer, PickingTaskManager* pRTC){
	m_MotionGeneratorService = pCorbaConsumer;
	m_rtc = pRTC;
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
  return m_MotionGeneratorService->_ptr()->getCurrentRobotJointAngles(jointAngles);
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::followManipPlan(const Manipulation::ManipulationPlan& manipPlan){
  Manipulation::ManipulationPlan_var plan;
  plan = new Manipulation::ManipulationPlan(manipPlan);

  createFollowingThread(plan);

  while(true){
    if(isDisconnected){
      while(!isDisconnected){
        m_rtc->refreshManipPlan(plan);
        createFollowingThread(plan);
      }
    }
    if(m_result){
      return m_result;
    }
  }

  //RETURN_ID::ERROR_UNKNOWN;
  return m_result;
}

void MotionGeneratorServiceDecorator::createFollowingThread(const Manipulation::ManipulationPlan& manipPlan){
  try {
    std::thread following(&MotionGeneratorServiceDecorator::callFollowManipPlan, this, manipPlan);
    following.detach();
  } catch (std::exception &ex) {
    std::cerr << ex.what() << std::endl;
  }
}
  
void MotionGeneratorServiceDecorator::callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan){
  m_result = m_MotionGeneratorService->_ptr()->followManipPlan(manipPlan);
}

// End of example implementational code



