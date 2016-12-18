#ifndef CUIAPP_H
#define CUIAPP_H

#include <fstream>
#include <sstream>
#include "TrajectoryPlannerStub.h"

class CUIApp{
public:
	CUIApp();
	~CUIApp();
	
	void showObjectIdentifier(Manipulation::ObjectIdentifier m_objectID);
	void showObjectInfo(Manipulation::ObjectInfo m_objInfo);
	void showRobotIdentifier(Manipulation::RobotIdentifier m_robotID);
	Manipulation::RobotJointInfo* m_robotJoint;

	Manipulation::RobotJointInfo* m_startRobotJointInfo;
	Manipulation::RobotJointInfo* m_goalRobotJointInfo;
	Manipulation::ManipulationPlan* m_manipPlan;

}

#endif // CUIAPP_H