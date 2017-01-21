// -*-C++-*-
#include "MotionGeneratorServiceDecorator.h"
#include <thread>
#include <exception>

MotionGeneratorServiceDecorator::MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pPortBase, SimplePathFollower* pRTC){
	m_base = pPortBase;
	m_rtc = pRTC;
}


Manipulation::ReturnValue* MotionGeneratorServiceDecorator::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
	return m_base->getCurrentRobotJointAngles(jointAngles)
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::followManipPlan(const Manipulation::ManipulationPlan& manipPlan){
  Manipulation::ManipulationPlan_var plan;
  plan = manipPlan;
  createFollowingThread(plan);

  while(true){
    if(isDisconnected){
      createFollowingThread(m_rtc->refreshManipPlan(manipPlan));
    }
    if(result){
      return m_result;
    }
  }
    return Manipulation::UNKOWN_ERROR;
}

void createFollowingThread(const Manipulation::ManipulationPlan& manipPlan){
  try {
    std::thread following(followManipPlan(plan));
    following.detach();
  } catch (std::exception &ex) {
    std::cerr << ex.what() << std::endl;
  }
}
  
void followManipPlan(manipPlan){
    m_result = m_base->forManipPlan(manipPlan);
}

// End of example implementational code



