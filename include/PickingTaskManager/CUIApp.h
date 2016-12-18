#ifndef CUIAPP_H
#define CUIAPP_H

#include <fstream>
#include <sstream>
#include "TrajectoryPlannerStub.h"

#include "PickingTaskManager.h"
class PickingTaskManager;

class CUIApp{
public:
	CUIApp(PickingTaskManager* compPtr);
	~CUIApp();

	void detectObj();
	void searchMotionPlan();
	void generateMotionPlan();
	void showParams();

	//for debugging only, delete this function before release
	void setSampleManipPlan();

private:
	Manipulation::ObjectIdentifier* m_objectID;
	Manipulation::ObjectInfo* m_objInfo;

	Manipulation::RobotIdentifier* m_robotID;
	Manipulation::RobotJointInfo* m_robotJoint;

	Manipulation::RobotJointInfo* m_startRobotJointInfo;
	Manipulation::RobotJointInfo* m_goalRobotJointInfo;
	Manipulation::ManipulationPlan* m_manipPlan;
	
	PickingTaskManager* m_rtc;
}

#endif // CUIAPP_H