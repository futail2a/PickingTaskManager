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

	m_rtc = compPtr;
}

void CUIApp::detectObj(){
	std::cout << "--Object Detection--" << std::endl;
	std::cout << "Please input target object name" << std::endl;

	std::string name;
	std::cin >> name;

	m_objectID->name = CORBA::string_dup(name.c_str());

	try{
		m_rtc->callDetectObject((*m_objectID), m_objInfo);
	}catch(CORBA::SystemException &e){
		std::cout << "Port Not Connected" <<std::endl;
	}

	//CORBA::string_free(m_objectID->name);
}

void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig--" << std::endl;

	std::cout << "Solve grasping position" << std::endl;
	m_rtc->callSolveInverseKinematics((*m_targetPose), (*m_startRobotJointAngles), m_currentRobotJointAngles);

	m_rtc->callGetCurrentRobotJointAngles(m_currentRobotJointAngles);
	m_rtc->callPlanManipulation((*m_robotID), (*m_currentRobotJointAngles), (*m_goalRobotJointAngles), m_manipPlan);

}

void CUIApp::generateMotionPlan(){
	std::cout << "--Motion Generation--" << std::endl;
	m_rtc->callFollowManipPlan((*m_manipPlan));

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
