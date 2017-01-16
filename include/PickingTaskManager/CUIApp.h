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

	Manipulation::ManipulationPlan_var m_manipPlan;
	Manipulation::ManipulationPlan_var m_replacingPlan;
	Manipulation::EndEffectorPose_var  m_targetPose;
	
	PickingTaskManager* m_rtc;

	Manipulation::ManipulationPlan_var getInversePlan(const Manipulation::ManipulationPlan& path);

};

#endif // CUIAPP_H
