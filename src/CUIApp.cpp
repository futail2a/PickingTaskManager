#include "CUIApp.h"
#include "PickingTaskManager.h"

CUIApp::CUIApp(PickingTaskManager* compPtr)
{
	m_objectID = new Manipulation::ObjectIdentifier();
	m_objInfo = new Manipulation::ObjectInfo();

	m_robotID = new Manipulation::RobotIdentifier();
	m_robotJoint = new Manipulation::RobotJointInfo();

	m_startRobotJointAngles = new Manipulation::JointAngleSeq();
	m_goalRobotJointAngles = new Manipulation::JointAngleSeq();
	m_manipPlan = new Manipulation::ManipulationPlan();
	m_targetPose = new Manipulation::EndEffectorPose;

	m_rtc = compPtr;
}

void CUIApp::detectObj(){
	std::cout << "--Object Detection--" << std::endl;
	std::cout << "Please input target object name" << std::endl;

	std::string name;
	std::cin >> name;

	m_objectID->name = CORBA::string_dup(name.c_str());

	try{
		m_rtc->callDetectObject(m_objectID, m_objInfo);
		std::cout <<"obj x:"<< m_objInfo->pose.position.x<<" obj y:" <<m_objInfo->pose.position.y <<" obj z:"<< m_objInfo->pose.position.z << std::endl;
		std::cout <<"obj p:"<< m_objInfo->pose.orientation.p<<" obj r:" <<m_objInfo->pose.orientation.r <<" obj y:"<< m_objInfo->pose.orientation.y << std::endl;
	}catch(CORBA::SystemException &e){
		std::cout << "Port Not Connected" <<std::endl;
	}
}

void CUIApp::determineApproachPose(){
	std::cout << "--Generate EndEffector Pose--" << std::endl;
        m_rtc->callGetApproachOrientation(m_objInfo, m_targetPose);
        /*
        m_targetPose->pose.position.x = m_objInfo->pose.position.x ;
	m_targetPose->pose.position.y = m_objInfo->pose.position.y ;
	m_targetPose->pose.position.z = m_objInfo->pose.position.z ;
	m_targetPose->pose.orientation.p = m_objInfo->pose.orientation.p ;
	m_targetPose->pose.orientation.r = m_objInfo->pose.orientation.r ;
	m_targetPose->pose.orientation.y = m_objInfo->pose.orientation.y ;
	*/
	
	std::cout <<"ee x:"<< m_targetPose->pose.position.x<<" ee y:" <<m_targetPose->pose.position.y <<" ee z:"<< m_targetPose->pose.position.z << std::endl;
	std::cout <<"ee p:"<< m_targetPose->pose.orientation.p<<" ee r:" <<m_targetPose->pose.orientation.r <<" ee y:"<< m_targetPose->pose.orientation.y << std::endl;
}

void CUIApp::solveKinematics(){
	std::cout << "--Solve Inverse Kinematics--" << std::endl;
	m_rtc->callGetCurrentRobotJointAngles(m_startRobotJointAngles);
        /*
	m_currentRobotJointAngles->length(7);
	m_currentRobotJointAngles[0].data=0.0;
	m_currentRobotJointAngles[1].data=0.0;
	m_currentRobotJointAngles[2].data=0.5;
	m_currentRobotJointAngles[3].data=0.0;
	m_currentRobotJointAngles[4].data=1.1;
	m_currentRobotJointAngles[5].data=0.0;
	m_currentRobotJointAngles[6].data=0.9;
	*/
  
	m_rtc->callSolveKinematics(m_targetPose, m_startRobotJointAngles, m_goalRobotJointAngles);
}


void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig--" << std::endl;
	/*
	m_startRobotJointAngles->length(7);
	m_startRobotJointAngles[0].data=0.0;
	m_startRobotJointAngles[1].data=0.0;
	m_startRobotJointAngles[2].data=0.5;
	m_startRobotJointAngles[3].data=0.0;
	m_startRobotJointAngles[4].data=1.1;
	m_startRobotJointAngles[5].data=0.0;
	m_startRobotJointAngles[6].data=0.9;

	m_goalRobotJointAngles->length(7);
	m_goalRobotJointAngles[0].data=0.0;
	m_goalRobotJointAngles[1].data=1.10;
	m_goalRobotJointAngles[2].data=0.6;
	m_goalRobotJointAngles[3].data=0.14;
	m_goalRobotJointAngles[4].data=0.341;
	m_goalRobotJointAngles[5].data=0.0;
	m_goalRobotJointAngles[6].data=0.9;
	*/
	
	m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_goalRobotJointAngles, m_manipPlan);
	
	std::cout <<m_manipPlan->manipPath.length() << std::endl;
	for(int i =0;i<m_manipPlan->manipPath.length(); i++){
	  for(int j=0;j<m_manipPlan->manipPath[i].length();j++){
	    std::cout << m_manipPlan->manipPath[i][j].data << " ";
	  }
	  std::cout <<std::endl;
	}
}

void CUIApp::generateMotionPlan(){
	std::cout << "--Motion Generation--" << std::endl;
	m_rtc->callFollowManipPlan(m_manipPlan);
	//TODO:open and close grip
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

/*
void CUIApp::setSampleManipPlan(){
	std::cout << "Generate motion from a csv file" << std::endl;

	std::string str;
	std::ifstream ifs("sampleManipPath.csv");

	if (!ifs){
		std::cout << "error" << std::endl;
	}

	while (getline(ifs, str))
	{
		std::string tmp;
		std::istringstream stream(str);
		Manipulation::RobotJointInfo posture;

		while (getline(stream, tmp, ','))
		{
			Manipulation::JointInfo joint;
			joint.jointAngle = std::stod(tmp);

			CORBA::ULong len = posture.jointInfoSeq.length();
			posture.jointInfoSeq.length(len + 1);
			posture.jointInfoSeq[len] = joint;
		}

		CORBA::ULong len = m_manipPlan->robotJointInfoSeq.length();
		m_manipPlan->robotJointInfoSeq.length(len + 1);
		m_manipPlan->robotJointInfoSeq[len] = posture;
	}

	//std::cout << "Test solving kinematics.." << std::endl;
	//m_KinematicsSolverService->solveInverseKinematics((*m_objInfo), m_goalRobotJointInfo);
}
*/
