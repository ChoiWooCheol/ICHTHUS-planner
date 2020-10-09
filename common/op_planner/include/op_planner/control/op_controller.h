
/// \file op_controller.h
/// \brief PID based trajectory follower and velocity controller
/// \author Hatem Darweesh
/// \date July 04, 2020


#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_
#include "op_planner/RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlannerCommonDef.h"

namespace PlannerHNS
{

#define MAX_DECELERSTION -3*9.8 // m/s*s , max deceleration value is 3G

class ExtendedVehicleState : public VehicleState
{
public:
	double steer_torque = 0;
	double accel_stroke = 0;
	double brake_stroke = 0;
};

class MotionControl
{
public:
	MotionControl();
	virtual ~MotionControl();

	void UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path);

	int SteerControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
			const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
			const double& lateralErr, double& desiredSteerTorque);

	int VeclocityControllerUpdateTwoPID(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift);

	int VeclocityControllerUpdateForOpenPlannerInternalACC(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift);

	void Init(const PlannerHNS::ControllerParams& params, const PlannerHNS::CAR_BASIC_INFO& vehicleInfo, bool bEnableLogs = false, bool bCalibration = false);

	PlannerHNS::ExtendedVehicleState DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
				const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
				const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory);

	void ResetLogTime(const double& v0, const double& v1);

	//Testing Points
	PlannerHNS::WayPoint m_PerpendicularPoint; // on track point, parallel to vehicle
	PlannerHNS::WayPoint m_FollowMePoint;
	double m_FollowingDistance;
	std::string m_ExperimentFolderName;
	bool m_bUseInternalOpACC;

private:
	PlannerHNS::ControllerParams m_Params;
	PlannerHNS::CAR_BASIC_INFO m_VehicleInfo;
	std::vector<PlannerHNS::WayPoint> m_Path;
	double m_PrevDesiredTorque; // control output
	PlannerHNS::BehaviorState m_PrevBehaviorStatus;
	double m_PrevFollowDistance;
	std::vector<double> m_RelativeSpeeds;
	double m_AverageRelativeSpeed;
	double m_PrevAngleError;
	PlannerHNS::ExtendedVehicleState m_PrevDesiredState;
	double m_ffEstimatedVelocity;
	double m_PredictedVelMinusRealVel;

	/**
	 * Log Information
	 */
	int m_iPrevWayPoint;
	double m_LateralError;
	double m_TargetAngle;
	double m_TargetSpeed;
	double m_PrevSpeedError;
	double m_PrevDistanceError;
	double m_PrevSpeed;
	double m_InstantAcceleration;
	double m_DesiredDistance;
	double m_DesiredSafeDistance;
	double m_AverageAcceleration;
	double m_TotalAcceleration;
	double m_AccelerationSum;
	int m_nAccelerations;
	double m_PredictedRelativeSpeed;
	double m_TargetAcceleration;

	UtilityHNS::PIDController m_pidSteer;
	UtilityHNS::LowpassFilter m_lowpassSteer;

	UtilityHNS::PIDController m_pidAccel;
	UtilityHNS::PIDController m_pidFollow;
	UtilityHNS::PIDController m_pidBrake;

	bool m_bEnableLog;
	std::vector<std::string> m_LogData;
	std::vector<std::string> m_LogSteerPIDData;
	std::vector<std::string> m_LogFollowPIDData;
	std::vector<std::string> m_LogAccelerationPIDData;
	std::vector<std::string> m_LogBrakingPIDData;

	//Steering and Velocity Calibration Global Variables
	bool m_bCalibrationMode;
	int	m_iNextTest;
	std::vector<std::string> m_SteerCalibrationData;
	std::vector<std::string> m_VelocityCalibrationData;
	PlannerHNS::VehicleState m_prevCurrState_steer;
	PlannerHNS::VehicleState m_prevDesiredState_steer;
	PlannerHNS::VehicleState m_prevCurrState_vel;
	PlannerHNS::VehicleState m_prevDesiredState_vel;
	struct timespec m_SteerDelayTimer;
	struct timespec m_VelocityDelayTimer;
	struct timespec m_LogTimer;
	std::vector<std::pair<double, double> > m_CalibrationRunList;

	bool FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
			const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
			double& lateral_err, double& follow_distance);

	int SteerControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& steerd);

	void PredictMotion(double& x, double &y, double& heading, double steering, double velocity,
			double wheelbase, double time_elapsed);

	double PredictVelocity(double v0, double v_d, double accel_stroke, double brake_stroke, double time_elapsed);

	double GetPID_LinearChange(double minVal, double maxVal, double speedMax, double currSpeed);

	void CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& vel_d, double& acc_d, double& distance_d, double& safe_d);

	void LogCalibrationData(const PlannerHNS::VehicleState& currState,const PlannerHNS::VehicleState& desiredState);
	void InitCalibration();
	void CalibrationStep(const double& dt, const PlannerHNS::VehicleState& CurrStatus, double& desiredSteer, double& desiredVelocity);
	void CoordinateAscent(double tolerance, PID_CONST& pOut);
	bool CalcAvgRelativeSpeedFromDistance(const double& dt, const PlannerHNS::BehaviorState& CurrBehavior, double avg_relative_speed);
};

} /* namespace PlannerHNS */

#endif /* MOTION_CONTROLLER_H_ */
