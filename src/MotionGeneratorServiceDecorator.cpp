// -*-C++-*-
#include "MotionGeneratorServiceDecorator.h"
#include <thread>
#include <exception>

struct threadAborted{};

MotionGeneratorServiceDecorator::MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pCorbaConsumer, PickingTaskManager* pRTC){
	m_MotionGeneratorService = pCorbaConsumer;
	m_rtc = pRTC;
	m_result = new Manipulation::ReturnValue();
    m_result->returnID = Manipulation::ERROR_UNKNOWN;
    m_result->message = CORBA::string_dup("No reply");
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
  return m_MotionGeneratorService->_ptr()->getCurrentRobotJointAngles(jointAngles);
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::followManipPlan(const Manipulation::ManipulationPlan& manipPlan){

  std::cout << "Create Thread" << std::endl;
  std::thread following(&MotionGeneratorServiceDecorator::callFollowManipPlan, this, manipPlan);

  while(true){
    if(isPortDisconnected){
      std::cout <<"Port was Disconnected"<<std::endl;
      Manipulation::ManipulationPlan_var plan;
      plan = new Manipulation::ManipulationPlan();

      while(true){
	    std::cout <<"Check port connection:"<<isPortDisconnected<<std::endl;
	    if(!isPortDisconnected){
	      std::cout<<"Retrying.."<<std::endl;
          m_rtc->refreshManipPlan(manipPlan, plan);

	      std::cout << "Create Thread" << std::endl;
	      std::thread following(&MotionGeneratorServiceDecorator::callFollowManipPlan, this, plan);
	      break;
        }
      }
    }
    
    if(m_result->returnID==Manipulation::OK){
      std::cout <<"RPC successed"<<std::endl;
      following.join();
      return m_result._retn();
    }
  }

  return m_result._retn();
}

//not used
void MotionGeneratorServiceDecorator::createFollowingThread(const Manipulation::ManipulationPlan& manipPlan){
  std::cout << "Create Thread" << std::endl;
    try {
    std::thread following(&MotionGeneratorServiceDecorator::callFollowManipPlan, this, manipPlan);
    following.join();
    } catch (std::exception &ex) {
    std::cerr << ex.what() << std::endl;
    }
}
  
int  MotionGeneratorServiceDecorator::callFollowManipPlan(const Manipulation::ManipulationPlan& manipPlan){
  try{
    m_result = m_MotionGeneratorService->_ptr()->followManipPlan(manipPlan);
  }catch(CORBA::Exception& e) {
    std::cout <<"RPC failed"<<std::endl;
    return 1;
  }
  return 0;
}

// End of example implementational code



