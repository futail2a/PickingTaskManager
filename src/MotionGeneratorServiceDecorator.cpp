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
  Manipulation::ManipulationPlan_var plan;
  plan = new Manipulation::ManipulationPlan(manipPlan);

  createFollowingThread(plan);

  while(true){
    if(isPortDisconnected){
      std::cout <<"Port was Disconnected"<<std::endl;	  
      while(true){
	std::cout <<"Check port connection:"<<isPortDisconnected<<std::endl;	 
	  if(!isPortDisconnected){
	    std::cout<<"Retrying.."<<std::endl;
	    sleep(10);
            //m_rtc->refreshManipPlan(plan);
            createFollowingThread(plan);
	    break;
          }
        }
    }
    
    if(m_result->returnID==0){
      std::cout <<"RPC successed"<<std::endl;      
      return m_result._retn();
    }
    }

  //RETURN_ID::ERROR_UNKNOWN;
  return m_result._retn();
}

void MotionGeneratorServiceDecorator::createFollowingThread(const Manipulation::ManipulationPlan& manipPlan){
  std::cout << "Create Thread" << std::endl;
    try {
    std::thread following(&MotionGeneratorServiceDecorator::callFollowManipPlan, this, manipPlan);
    following.detach();
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
   // throw threadAborted{};
  }
  return 0;
}

// End of example implementational code



