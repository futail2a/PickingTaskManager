#ifndef CUIAPP_H
#define CUIAPP_H

#include <fstream>
#include <sstream>
#include "TrajectoryPlannerStub.h"

class PickingTaskManager;

class CUIApp{
public:
	CUIApp(PickingTaskManager* compPtr);
	~CUIApp();

	void detectObj();
	void determineApproachPose();
	void solveKinematics();
	void searchMotionPlan();
	void generateMotionPlan();
	void showParams();
	void searchReplacingPlan();
	void goStartPosition(double j[]);

	//for debugging only, delete this function before release
	//void setSampleManipPlan();

private:
	Manipulation::ObjectIdentifier_var m_objectID;
	Manipulation::ObjectInfo_var m_objInfo;

	Manipulation::RobotIdentifier_var m_robotID;
	Manipulation::RobotJointInfo_var m_robotJoint;

	Manipulation::JointAngleSeq_var m_startRobotJointAngles;
	Manipulation::JointAngleSeq_var m_goalRobotJointAngles;
	Manipulation::JointAngleSeq_var m_replacingRobotJointAngles;

	Manipulation::JointAngleSeq_var m_prestartRobotJointAngles;
	Manipulation::JointAngleSeq_var m_pregoalRobotJointAngles;

	
	Manipulation::ManipulationPlan_var m_manipPlan;
	Manipulation::ManipulationPlan_var m_invManipPlan;

	Manipulation::ManipulationPlan_var m_preManipPlan;
	Manipulation::ManipulationPlan_var m_invPreManipPlan;
	

	Manipulation::ManipulationPlan_var m_replacingPlan;
	Manipulation::ManipulationPlan_var m_invReplacingPlan;


	Manipulation::EndEffectorPose_var  m_targetPose;
	Manipulation::EndEffectorPose_var  m_preTargetPose;

	

	PickingTaskManager* m_rtc;

	Manipulation::ManipulationPlan* inversePlan(const Manipulation::ManipulationPlan& path);

};

#endif // CUIAPP_H
