// -*-C++-*-
#ifndef MGDECORATOR_H
#define MGDECORATOR_H

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSVC_impl.h"
#include "PickingTaskManager"
class PickingTaskManager;

class MotionGeneratorServiceDecorator : public RTC::CorbaConsumer<Manipulation::MotionGeneratorService>{
private:
  PickingTaskManager* m_rtc;
  RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* m_base;
  bool isDisconnected = false;
  Manipulation::ReturnValue* m_result;
  
public :
  MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pPortBase, SimplePathFollower* pRTC);
  ~MotionGeneratorServiceDecorator();
		 	
  Manipulation::ReturnValue* getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles);

  Manipulation::ReturnValue* followManipPlan(const Manipulation::ManipulationPlan& manipPlan);

  void connectionIs(bool b){isDisconnected=b;}
};


#endif // MGDECORATOR_H
