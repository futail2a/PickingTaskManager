#include "CUIApp.h"
#include "PickingTaskManager.h"

CUIApp::CUIApp(PickingTaskManager* compPtr)
{
	m_objectID = new Manipulation::ObjectIdentifier();
	m_objInfo = new Manipulation::ObjectInfo();

	m_robotID = new Manipulation::RobotIdentifier();
	m_robotJoint = new Manipulation::RobotJointInfo();

	m_currentRobotJointAngles = new Manipulation::JointAngleSeq();
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
	m_rtc->callGetApproachOrientation(m_objInfo, m_targetPose);
	std::cout <<"ee x:"<< m_targetPose->pose.position.x<<" ee y:" <<m_targetPose->pose.position.y <<" ee z:"<< m_targetPose->pose.position.z << std::endl;
	std::cout <<"ee p:"<< m_targetPose->pose.orientation.p<<" ee r:" <<m_targetPose->pose.orientation.r <<" ee y:"<< m_targetPose->pose.orientation.y << std::endl;
}

void CUIApp::solveKinematics(){
	//m_currentRobotJointAngles->length(7);
	m_rtc->callGetCurrentRobotJointAngles(m_currentRobotJointAngles);
	m_rtc->callSolveKinematics(m_targetPose, m_currentRobotJointAngles, m_startRobotJointAngles);
}


void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig--" << std::endl;

	m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_goalRobotJointAngles, m_manipPlan);
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
			std::cout << m_manipPlan->manipPath[i][j].data << " ";
		}
	std::cout << std::endl;
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
