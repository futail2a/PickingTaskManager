// -*-C++-*-
#include "MotionGeneratorServiceDecorator.h"
#include <thread>
#include <exception>

struct threadAborted{};

MotionGeneratorServiceDecorator::MotionGeneratorServiceDecorator(RTC::CorbaConsumer<Manipulation::MotionGeneratorService>* pCorbaConsumer, PickingTaskManager* pRTC){
	m_MotionGeneratorService = pCorbaConsumer;
	m_rtc = pRTC;
	m_result = new Manipulation::ReturnValue();
    m_result->id = Manipulation::ERROR_UNKNOWN;
    m_result->message = CORBA::string_dup("No reply");
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles){
  return m_MotionGeneratorService->_ptr()->getCurrentRobotJointAngles(jointAngles);
}

Manipulation::ReturnValue* MotionGeneratorServiceDecorator::followManipPlan(const Manipulation::ManipulationPlan& manipPlan){
  try{
	  m_result = m_MotionGeneratorService->_ptr()->followManipPlan(manipPlan);
  }catch(CORBA::Exception& e) {
	  std::cout <<"RPC failed"<<std::endl;

	  Manipulation::ManipulationPlan_var plan;
	  plan = new Manipulation::ManipulationPlan();
      plan->robotID.name = CORBA::string_dup("orochi");

	  while(true){
		  std::cout <<"Check port connection:"<<isPortDisconnected<<std::endl;
		  if(!isPortDisconnected){
		      std::cout<<"Retrying.."<<std::endl;
		      sleep(10);
		      m_rtc->refreshManipPlan(manipPlan, plan);

		  	std::cout << "Refreshed plan"<<std::endl;
		  	std::cout << plan ->manipPath.length()<<std::endl;
		  	for(int i =0;i<plan ->manipPath.length(); i++){
		  	   for(int j=0;j<plan ->manipPath[i].length();j++){
		  	      std::cout << plan ->manipPath[i][j].data << " ";
		  	   }
		  	   std::cout <<std::endl;
		  	}

    		  std::cout << "Restart" << std::endl;
	    	  m_result = m_MotionGeneratorService->_ptr()->followManipPlan(plan);
	    	  break;
		  }
	  }
  }

  return m_result._retn();
}

/*
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
  }
  return 0;
}
*/

// End of example implementational code



