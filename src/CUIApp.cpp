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
	m_replacingRobotJointAngles = new Manipulation::JointAngleSeq();

	m_prestartRobotJointAngles = new Manipulation::JointAngleSeq();
	m_pregoalRobotJointAngles = new Manipulation::JointAngleSeq();
	m_preManipPlan = new Manipulation::ManipulationPlan();
	
	m_manipPlan = new Manipulation::ManipulationPlan();
	m_invManipPlan = new Manipulation::ManipulationPlan();
	m_invReplacingPlan = new Manipulation::ManipulationPlan();
      	m_preManipPlan = new Manipulation::ManipulationPlan();
     

	m_replacingPlan = new Manipulation::ManipulationPlan();
	m_targetPose = new Manipulation::EndEffectorPose();
	//m_preTargetPose = new Manipulation::EndEffectorPose;


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
    m_preTargetPose=new Manipulation::EndEffectorPose(m_targetPose);
    m_preTargetPose->pose.position.x-=0.15;
    
	std::cout <<"ee x:"<< m_targetPose->pose.position.x<<" ee y:" <<m_targetPose->pose.position.y <<" ee z:"<< m_targetPose->pose.position.z << std::endl;
	std::cout <<"ee p:"<< m_targetPose->pose.orientation.p<<" ee r:" <<m_targetPose->pose.orientation.r <<" ee y:"<< m_targetPose->pose.orientation.y << std::endl;

std::cout <<"pre x:"<< m_preTargetPose->pose.position.x<<" ee y:" <<m_preTargetPose->pose.position.y <<" ee z:"<< m_preTargetPose->pose.position.z << std::endl;

}

void CUIApp::solveKinematics(){
	std::cout << "--Solve Inverse Kinematics--" << std::endl;
	m_rtc->callGetCurrentRobotJointAngles(m_startRobotJointAngles);

	Manipulation::ReturnValue_var resultPreKinematics = new Manipulation::ReturnValue();
	resultPreKinematics = m_rtc->callSolveKinematics(m_preTargetPose, m_startRobotJointAngles, m_pregoalRobotJointAngles);
	if (resultPreKinematics->returnID==0){       
	    std::cout <<m_rtc->callSolveKinematics(m_targetPose, m_pregoalRobotJointAngles, m_goalRobotJointAngles)->message<< std::endl;
	  }
	else {
	  std::cout << resultPreKinematics->message << std::endl;
	}

}


void CUIApp::searchMotionPlan(){
	std::cout << "--Motion Plannig: Picking--" << std::endl;

	for(int i=0;i<6;i++){
		std::cout << m_startRobotJointAngles[i].data<< std::endl;
	}
	for(int i=0;i<6;i++){
		std::cout << m_goalRobotJointAngles[i].data<< std::endl;
	}


	
	m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_pregoalRobotJointAngles, m_preManipPlan);
	//m_rtc->callPlanManipulation(m_robotID, m_goalRobotJointAngles, m_startRobotJointAngles, m_invManipPlan);
	m_rtc->callPlanManipulation(m_robotID, m_pregoalRobotJointAngles, m_goalRobotJointAngles, m_manipPlan);

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
	m_replacingRobotJointAngles[0].data=-1.4;
	m_replacingRobotJointAngles[1].data=0.9;
	m_replacingRobotJointAngles[2].data=0.9;
	m_replacingRobotJointAngles[3].data=0.0;
	m_replacingRobotJointAngles[4].data=-0.5;
	m_replacingRobotJointAngles[5].data=0.0;


	std::cout << "--Motion Plannig: Replacing--" << std::endl;
	//m_robotID->name = CORBA::string_dup("orochi");
	m_rtc->callPlanManipulation(m_robotID, m_startRobotJointAngles, m_replacingRobotJointAngles, m_replacingPlan);
	//m_rtc->callPlanManipulation(m_robotID, m_replacingRobotJointAngles, m_startRobotJointAngles, m_invReplacingPlan);

	std::cout <<"Replacing motion plan:" << std::endl;
	std::cout <<"length: " << m_replacingPlan->manipPath.length() << std::endl;
	for(int i =0;i<m_replacingPlan ->manipPath.length(); i++){
	  for(int j=0;j<m_replacingPlan ->manipPath[i].length();j++){
		std::cout << m_replacingPlan ->manipPath[i][j].data << " ";
	  }
	  std::cout <<std::endl;
	}

}

void CUIApp::generateMotionPlan(){
	std::cout << "--Motion Generation--" << std::endl;

	JARA_ARM::CarPosWithElbow targetPos;
	targetPos.elbow = 0;
	targetPos.carPos[0][0] = 1;
	targetPos.carPos[0][1] = 0;
	targetPos.carPos[0][2] = 0;
	targetPos.carPos[0][3] = 0;
	targetPos.carPos[1][0] = 0;
	targetPos.carPos[1][1] = 1;
	targetPos.carPos[1][2] = 0;
	targetPos.carPos[1][3] = 0;
	targetPos.carPos[2][0] = 0;
	targetPos.carPos[2][1] = 0;
	targetPos.carPos[2][2] = 1;
	targetPos.carPos[2][3] = 0;

	std::cout << "--Move to Target Object--" << std::endl;
	m_rtc->callFollowManipPlan(m_preManipPlan);

	std::cout << "Approaching" << std::endl;
        //targetPos.carPos[0][3] = +100;
	//m_rtc->callMovePTPCartesianRel(targetPos);
	m_rtc->callFollowManipPlan(m_manipPlan);
	
	std::cout << "--Try Graspping--" << std::endl;
	m_rtc->callMoveGripper(70);
	sleep(3);
	//targetPos.carPos[0][3] = -100;
	//m_rtc->callMovePTPCartesianRel(targetPos);
	
	std::cout << "--Move to Init Pose--" << std::endl;
	
	m_invManipPlan = inversePlan(m_manipPlan);
	m_rtc->callFollowManipPlan(m_invManipPlan);

	m_invPreManipPlan = inversePlan(m_preManipPlan);
	m_rtc->callFollowManipPlan(m_invPreManipPlan);
	
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
	tmp->manipPath.length(plan.manipPath.length());

        //m_rtc->callPlanManipulation(m_robotID, m_goalRobotJointAngles, m_startRobotJointAngles, m_manipPlan);
	
	
	int k =0;
	for(int i=plan.manipPath.length()-1; i>=0;i--){
		tmp->manipPath[k].length(plan.manipPath[i].length());
		for(int j=0; j<plan.manipPath[i].length();j++){
			tmp->manipPath[k][j].data = plan.manipPath[i][j].data;
		}
		k+=1;
	}
	std::cout << "success calc inverse manip plan" << std::endl;
	return tmp._retn();
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

//void CUIApp::goHome(double j[]){
//	unsigned long spdRaion;
//	std::cout << "Enter joint spped [%]" << std::endl;
//	std::cin>>spdRaion;
//
//	std::cout << "Go home position with configuration parameters" << std::endl;
//
//	JARA_ARM::JointPos_var jpos = new JARA_ARM::JointPos();
//    jpos->length(6);
//	for (int i =0; i<6; i++){
//		jpos[i]=j[i];
//	}
//
//	m_rtc->callSetHome(jpos);
//	m_rtc->callGoHome();
//
//}

void CUIApp::goStartPosition(double home_j[]){
	unsigned long spdRation;
	std::cout << "Enter joint spped [%]" << std::endl;
	std::cin>>spdRation;

	std::cout << "Go home position with configuration parameters" << std::endl;
	m_rtc->callSetSpeedJoint(spdRation);
	JARA_ARM::JointPos_var jpos = new JARA_ARM::JointPos();
    jpos->length(6);
	for (int i =0; i<6; i++){
		jpos[i]=home_j[i];
		std::cout << home_j[i] << std::endl;
	}


	m_rtc->callMovePTPJointAbs(jpos);

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
