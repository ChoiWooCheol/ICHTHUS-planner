
/// \file op_acc.h
/// \brief PID based velocity controller
/// \author Hatem Darweesh
/// \date September 02, 2020


#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_
#include "op_planner/RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlannerCommonDef.h"

namespace PlannerHNS
{

class ACC
{
public:
	ACC();
	virtual ~ACC();

	void Init(const PlannerHNS::ControllerParams& params, const PlannerHNS::CAR_BASIC_INFO& vehicleInfo, bool bModelBased = true);

	PlannerHNS::VehicleState DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior, const PlannerHNS::VehicleState& vehicleState);

	std::string m_ExperimentFolderName;

private:
	PlannerHNS::ControllerParams m_Params;
	PlannerHNS::CAR_BASIC_INFO m_VehicleInfo;
	bool m_bModelBased;
	PlannerHNS::BehaviorState m_PrevBehaviorStatus;
	PlannerHNS::VehicleState m_PrevDesiredState;
	double m_PrevFollowDistance;
	std::vector<double> m_RelativeSpeeds;
	double m_AverageRelativeSpeed;

	UtilityHNS::PIDController m_pidAccel;
	UtilityHNS::PIDController m_pidFollow;
	UtilityHNS::PIDController m_pidBrake;


	void CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& vel_d, double& acc_d, double& distance_d, double& safe_d);

	int UpdateVelocity(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVel, PlannerHNS::SHIFT_POS& desiredShift);

	void UpdateVelocityModelBased(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVel, PlannerHNS::SHIFT_POS& desiredShift);

	bool CalcAvgRelativeSpeedFromDistance(const double& dt, const PlannerHNS::BehaviorState& CurrBehavior, double avg_relative_speed);
};

} /* namespace PlannerHNS */

#endif /* VELOCITY_CONTROLLER_H_ */
