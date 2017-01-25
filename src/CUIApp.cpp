#include "CUIApp.h"
#include "PickingTaskManager.h"
#include<fstream>
#include<iostream>

CUIApp::CUIApp(PickingTaskManager* compPtr)
{
	m_objectID = new Manipulation::ObjectIdentifier();
	m_objInfo = new Manipulation::ObjectInfo();

	m_robotID = new Manipulation::RobotIdentifier();
	m_robotJoint = new Manipulation::RobotJointInfo();

	m_startRobotJointAngles = new Manipulation::JointAngleSeq();
	m_goalRobotJointAngles = new Manipulation::JointAngleSeq();
	m_replacingRobotJointAngles = new Manipulation::JointAngleSeq();

	m_manipPlan = new Manipulation::ManipulationPlan();
	m_invManipPlan = new Manipulation::ManipulationPlan();
	m_invReplacingPlan = new Manipulation::ManipulationPlan();

	m_replacingPlan = new Manipulation::ManipulationPlan();
	m_targetPose = new Manipulation::EndEffectorPose;

	m_rtc = compPtr;
}

void CUIApp::detectObj(){
	std::cout << "--Object Detection--" << std::endl;
	std::cout << "Please input target object name" << std::endl;

	std::string name;
	std::cin >> name;

	m_objectID->name = CORBA::string_dup(name.c_str());

	m_objInfo->objectID.name = CORBA::string_dup(name.c_str());
	m_objInfo->pose.position.x =0.0;
    m_objInfo->pose.position.y =0.0;
	m_objInfo->pose.position.z =0.0;
	m_objInfo->pose.orientation.p =0.0;
	m_objInfo->pose.orientation.r =0.0;
	m_objInfo->pose.orientation.y =0.0;

	try{
		m_rtc->callDetectObject(m_objectID, m_objInfo);
	}catch(CORBA::SystemException &e){
		std::cout << "Port Not Connected" <<std::endl;
	}

		std::cout <<"obj x:"<< m_objInfo->pose.position.x<<" obj y:" <<m_objInfo->pose.position.y <<" obj z:"<< m_objInfo->pose.position.z << std::endl;
		std::cout <<"obj p:"<< m_objInfo->pose.orientation.p<<" obj r:" <<m_objInfo->pose.orientation.r <<" obj y:"<< m_objInfo->pose.orientation.y << std::endl;

	//CORBA::string_free(m_objectID->name);
}

void CUIApp::determineApproachPose(){
	std::cout << "--Generate EndEffector Pose--" << std::endl;
    m_rtc->callGetApproachOrientation(m_objInfo, m_targetPose);
	
	std::cout <<"ee x:"<< m_targetPose->pose.position.x<<" ee y:" <<m_targetPose->pose.position.y <<" ee z:"<< m_targetPose->pose.position.z << std::endl;
	std::cout <<"ee p:"<< m_targetPose->pose.orientation.p<<" ee r:" <<m_targetPose->pose.orientation.r <<" ee y:"<< m_targetPose->pose.orientation.y << std::endl;
}

void CUIApp::solveKinematics(){
	std::cout << "--Solve Inverse Kinematics--" << std::endl;
	m_rtc->callGetCurrentRobotJointAngles(m_startRobotJointAngles);
	std::cout << m_rtc->callSolveKinematics(m_targetPose, m_startRobotJointAngles, m_goalRobotJointAngles)->message << std::endl;
}


void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig: Picking--" << std::endl;
	for(int i=0;i<6;i++){
		std::cout << m_goalRobotJointAngles[i].data<< std::endl;
	}


	
	m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_goalRobotJointAngles, m_manipPlan);

	std::cout <<"Picking motion plan:" << std::endl;
	std::cout <<"length: " << m_manipPlan->manipPath.length() << std::endl;
	for(int i =0;i<m_manipPlan->manipPath.length(); i++){
	  for(int j=0;j<m_manipPlan->manipPath[i].length();j++){
	    std::cout << m_manipPlan->manipPath[i][j].data << " ";
	  }
	  std::cout <<std::endl;
	}
	searchReplacingPlan();
}

void  CUIApp::searchReplacingPlan(){

	//TODO: Define these parameters as configuration
	m_replacingRobotJointAngles->length(6);
	m_replacingRobotJointAngles[0].data=-1.5;
	m_replacingRobotJointAngles[1].data=1.0;
	m_replacingRobotJointAngles[2].data=1.0;
	m_replacingRobotJointAngles[3].data=0.0;
	m_replacingRobotJointAngles[4].data=-0.5;
	m_replacingRobotJointAngles[5].data=0.0;


	std::cout << "--Motion Plannig: Replacing--" << std::endl;
	//m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_replacingRobotJointAngles, m_replacingPlan);

	std::cout <<"Replacing motion plan:" << std::endl;
	std::cout <<"length: " << m_manipPlan->manipPath.length() << std::endl;
	for(int i =0;i<m_replacingPlan ->manipPath.length(); i++){
	  for(int j=0;j<m_replacingPlan ->manipPath[i].length();j++){
		std::cout << m_replacingPlan ->manipPath[i][j].data << " ";
	  }
	  std::cout <<std::endl;
	}

}

void CUIApp::generateMotionPlan(){
	std::cout << "--Motion Generation--" << std::endl;

	std::cout << "--Move to Target Object--" << std::endl;
	m_rtc->callFollowManipPlan(m_manipPlan);

	std::cout << "--Try Graspping--" << std::endl;
	m_rtc->callMoveGripper(70);
	//sleep(3);
	m_invManipPlan = inversePlan(m_manipPlan);
	m_rtc->callFollowManipPlan(m_invManipPlan);

	std::cout << "--Move to Cargo--" << std::endl;
	m_rtc->callFollowManipPlan(m_replacingPlan);
	m_rtc->callOpenGripper();

	std::cout << "--Move to Initial Pose--" << std::endl;
	m_invReplacingPlan = inversePlan(m_replacingPlan);
	m_rtc->callFollowManipPlan(m_invReplacingPlan);
}

Manipulation::ManipulationPlan* CUIApp::inversePlan(const Manipulation::ManipulationPlan& plan){
	Manipulation::ManipulationPlan_var tmp;
	tmp = new Manipulation::ManipulationPlan();
	tmp->manipPath.length(plan.manipPath.length()*6);

	int k =0;
	for(int i=plan.manipPath.length()-1; i>=0;i--){
		tmp->manipPath[k].length(plan.manipPath[i].length());
		for(int j=0; j<plan.manipPath[i].length();j++){
			tmp->manipPath[k][j].data = plan.manipPath[i][j].data;
		}
		k+=1;
	}
	return tmp._retn();
}

void CUIApp::debugReplication(){
	std::cout << "--Motion Debugging--" << std::endl;
    m_startRobotJointAngles->length(7);
    m_startRobotJointAngles[0].data=0.0;
    m_startRobotJointAngles[1].data=0.5;
    m_startRobotJointAngles[2].data=0.0;
    m_startRobotJointAngles[3].data=0.0;
    m_startRobotJointAngles[4].data=1.0;
    m_startRobotJointAngles[5].data=0.0;
    m_startRobotJointAngles[6].data=0.9;

    m_goalRobotJointAngles->length(7);
    m_goalRobotJointAngles[0].data=0.0;
    m_goalRobotJointAngles[1].data=0.3938;
    m_goalRobotJointAngles[2].data=1.9663;
    m_goalRobotJointAngles[3].data=0.0;
    m_goalRobotJointAngles[4].data=-0.749;
    m_goalRobotJointAngles[5].data=0.0;
    m_goalRobotJointAngles[6].data=0.9;

	std::cout << "--Motion Plannig--" << std::endl;
	m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_goalRobotJointAngles, m_manipPlan);
	//writeManipPlanIntoCSV();
	//setSampleManipPlan();

	std::cout << "--Start motion--" << std::endl;
	//m_rtc->callMoveGripper(70);
	m_rtc->callFollowManipPlan(m_manipPlan);


	std::cout << "--Start Inv motion--" << std::endl;
	//m_rtc->callOpenGripper();
	//sleep(3);
	//m_invManipPlan = inversePlan(m_manipPlan);
	//m_rtc->callFollowManipPlan(m_invManipPlan);


}

void CUIApp::showParams(){
	std::cout << "Current motion plan" << std::endl;
	for (int i = 0; i<m_manipPlan->manipPath.length(); i++){
		for (int j = 0; j<m_manipPlan->manipPath[i].length(); j++){
		  //std::cout << m_manipPlan->manipPath[i][j].data << " ";
		}
		//std::cout << std::endl;
	}
}

void CUIApp::writeManipPlanIntoCSV(){
  std::ofstream ofs("m_manipPlan.csv");
    for(int i=0;i<m_manipPlan->manipPath.length();i++){
      for(int j=0;j<m_manipPlan->manipPath[i].length()-1;j++){
          ofs << m_manipPlan->manipPath[i][j].data <<",";
      }
      ofs << m_manipPlan->manipPath[i][m_manipPlan->manipPath[i].length()-1].data << std::endl;
    }
      
}

void CUIApp::setSampleManipPlan(){
	std::cout << "Generate motion from a csv file" << std::endl;

	std::string str;
	std::ifstream ifs("sampleManipPath.csv");

	if (!ifs){
		std::cout << "error" << std::endl;
	}

	int i = 0;
	while (getline(ifs, str))
	{

		std::string tmp;
		std::istringstream stream(str);
		Manipulation::JointAngleSeq_var posture;

		m_manipPlan->manipPath.length(i+1);
                int j = 0;
		m_manipPlan->manipPath[i].length(6);
		while (getline(stream, tmp, ','))
		{
	          double  angle;

		  angle = std::stod(tmp);

		  //int len = m_manipPlan->manipPath[i].length();
		  //m_manipPlan->manipPath[i].length(len + 1);
		  m_manipPlan->manipPath[i][j].data = angle;
		  j++;
		}

		int len = m_manipPlan->manipPath.length();
		m_manipPlan->manipPath.length(len + 1);
		i++;
	}
	  std::cout << "finish" << std::endl;
}
