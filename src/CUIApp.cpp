#include "CUIApp.h"
#include "PickingTaskManager.h"

CUIApp::CUIApp(PickingTaskManager* compPtr)
{
	m_objectID = new Manipulation::ObjectIdentifier();
	m_objInfo = new Manipulation::ObjectInfo();

	m_robotID = new Manipulation::RobotIdentifier();
	m_robotJoint = new Manipulation::RobotJointInfo();

	m_startRobotJointInfo = new Manipulation::RobotJointInfo();
	m_goalRobotJointInfo = new Manipulation::RobotJointInfo();
	m_manipPlan = new Manipulation::ManipulationPlan();

	m_rtc = compPtr;
}

void CUIApp::detectObj(){
	std::cout << "--Object Detection--" << std::endl;
	std::cout << "Please input target object name" << std::endl;
	char* name;
	std::cin >> name;
	(*m_objectID).name = name;

	m_rtc->callDetectObject((*m_objectID), m_objInfo);
	m_rtc->callSolveInverseKinematics((*m_objInfo), m_goalRobotJointInfo);
}

void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig--" << std::endl;

	std::cout << "Solve grasping position" << std::endl;
	m_rtc->callSolveInverseKinematics((*m_objInfo), m_goalRobotJointInfo);

	m_rtc->callGetCurrentRobotJointInfo((*m_robotID), m_robotJoint);
	m_rtc->callPlanManipulation((*m_robotID), (*m_startRobotJointInfo), (*m_goalRobotJointInfo), m_manipPlan);


}

void CUIApp::generateMotionPlan(){
	std::cout << "--Motion Generation--" << std::endl;
	m_rtc->callFollowManipPlan((*m_manipPlan));

}

void CUIApp::showParams(){
	std::cout << "Current motion plan" << std::endl;
	for (int i = 0; i<m_manipPlan->robotJointInfoSeq.length(); i++){
		for (int j = 0; j<m_manipPlan->robotJointInfoSeq[i].jointInfoSeq.length(); j++){
			std::cout << m_manipPlan->robotJointInfoSeq[i].jointInfoSeq[j].jointAngle << " ";
		}
	std::cout << std::endl;
	}
}

//for debugging
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
