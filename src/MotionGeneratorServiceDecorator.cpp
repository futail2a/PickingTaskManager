// -*-C++-*-

#include "MotionGeneratorServiceDecorator.h"

MotionGeneratorServiceDecorator::MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pPortBase, SimplePathFollower* pRTC){
	m_base = pPortBase;
	m_rtc = pRTC;
}


Manipulation::ReturnValue* MotionGeneratorServiceDecorator::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
	return m_base->getCurrentRobotJointAngles(jointAngles)
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::followManipPlan(const Manipulation::ManipulationPlan& manipPlan){

	//TODO: impl disconn callback func
	m_rtc->refreshManipPlan(manipPlan)

	return m_base->followManipPlan(manipPlan)
}

// End of example implementational code



