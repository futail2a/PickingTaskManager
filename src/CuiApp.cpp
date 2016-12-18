#include <CuiApp.h>
	(*m_objectID)
	m_objInfo
	m_goalRobotJointInfo);

	m_MotionGeneratorService->getCurrentRobotJointInfo((*m_robotID), m_robotJoint);
	m_ManipulationPlannerService->planManipulation((*m_robotID), (*m_startRobotJointInfo), (*m_goalRobotJointInfo), m_manipPlan);
	break;


	void showManipPlan(){
		for (int i = 0; i<m_manipPlan->robotJointInfoSeq.length(); i++){
			for (int j = 0; j<m_manipPlan->robotJointInfoSeq[i].jointInfoSeq.length(); j++){
				std::cout << m_manipPlan->robotJointInfoSeq[i].jointInfoSeq[j].jointAngle << " ";
			}
		std::cout << std::endl;
		}
	}
