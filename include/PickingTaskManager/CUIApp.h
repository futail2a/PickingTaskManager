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

	//for debugging only, delete this function before release
	//void setSampleManipPlan();

private:
	Manipulation::ObjectIdentifier* m_objectID;
	Manipulation::ObjectInfo* m_objInfo;

	Manipulation::RobotIdentifier* m_robotID;
	Manipulation::RobotJointInfo* m_robotJoint;

	Manipulation::JointAngleSeq* m_currentRobotJointAngles;
	Manipulation::JointAngleSeq* m_startRobotJointAngles;
	Manipulation::JointAngleSeq* m_goalRobotJointAngles;
	Manipulation::ManipulationPlan* m_manipPlan;
	Manipulation::EndEffectorPose*  m_targetPose;
	
	PickingTaskManager* m_rtc;
};

#endif // CUIAPP_H
