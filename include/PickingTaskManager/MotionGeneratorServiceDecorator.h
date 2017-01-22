// -*-C++-*-
#ifndef MGDECORATOR_H
#define MGDECORATOR_H

#include <rtm/CorbaPort.h>
#include "PickingTaskManager.h"
class PickingTaskManager;

class MotionGeneratorServiceDecorator: public RTC::CorbaConsumer<Manipulation::MotionGeneratorService>{
private:
  PickingTaskManager* m_rtc;
  RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* m_base;
  bool isDisconnected = false;
  Manipulation::ReturnValue* m_result;
  void createFollowingThread(const Manipulation::ManipulationPlan& manipPlan);

  void callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan);
  
public:
  MotionGeneratorServiceDecorator();
  ~MotionGeneratorServiceDecorator();

  void init(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pPortBase, PickingTaskManager* pRTC);
		 	
  Manipulation::ReturnValue* getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles);

  Manipulation::ReturnValue* followManipPlan(const Manipulation::ManipulationPlan& manipPlan);

  void connectionIs(bool b){isDisconnected=b;}
};


#endif // MGDECORATOR_H
