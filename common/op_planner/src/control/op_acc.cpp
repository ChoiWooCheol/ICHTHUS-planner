
/// \file op_acc.cpp
/// \brief PID based velocity controller
/// \author Hatem Darweesh
/// \date September 02, 2020

#include "op_planner/control/op_acc.h"
#include "op_planner/PlanningHelpers.h"
#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace PlannerHNS
{

ACC::ACC()
{
	m_PrevFollowDistance = 0;
	m_AverageRelativeSpeed = 0;
	m_bModelBased = true;
}

void ACC::Init(const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo, bool bModelBased)
{
	m_bModelBased = bModelBased;
	m_Params = params;
	m_VehicleInfo = vehicleInfo;

	m_pidAccel.Init(m_Params.Accel_Gain.kP, m_Params.Accel_Gain.kI, m_Params.Accel_Gain.kD);
	m_pidAccel.Setlimit(m_VehicleInfo.max_accel_value, 0);

	m_pidFollow.Init(m_Params.Follow_Gain.kP, m_Params.Follow_Gain.kI, m_Params.Follow_Gain.kD);
	m_pidFollow.Setlimit(m_VehicleInfo.max_accel_value, 0);

	m_pidBrake.Init(m_Params.Brake_Gain.kP, m_Params.Brake_Gain.kI, m_Params.Brake_Gain.kD);
	m_pidBrake.Setlimit(m_VehicleInfo.max_brake_value, 0);
}

ACC::~ACC()
{
}

void ACC::CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& vel_d, double& acc_d, double& distance_d, double& safe_d)
{
	double desired_acceleration = 0, desired_velocity = 0;

	/**
	 * Forward State, No obstacle, just try to follow general max speed profile for the trajectory
	 */
	if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{
		if(CurrStatus.speed <= CurrBehavior.maxVelocity) //accelerate with half the max accel
		{
			desired_acceleration = m_VehicleInfo.max_acceleration;
		}
		else //brake with half the max decel
		{
			desired_acceleration = m_VehicleInfo.max_deceleration/2.0;
		}

		desired_velocity = CurrBehavior.maxVelocity;
	}
	/**
	 * Follow State, try to keep safe distance to the object in front, the faster the car in from goinf the larget this safe distance
	 */
	else if(CurrBehavior.state == FOLLOW_STATE) // maintain
	{
		distance_d = CurrBehavior.followDistance;
		safe_d = m_Params.min_safe_follow_distance;

		if(distance_d > safe_d) // maintain distance
		{
			desired_acceleration = m_VehicleInfo.max_acceleration;
		}
		else if(distance_d < safe_d*0.75) // too close , brake hard
		{
			desired_acceleration = m_VehicleInfo.max_deceleration;
		}
		else if(distance_d < safe_d) // use engine brake
		{
			desired_acceleration = m_VehicleInfo.max_deceleration/2.0;
		}

		desired_velocity = CurrBehavior.maxVelocity;

		/**
		 * TODO Add more safe follow distance according to the lead vehicle velocity
		 * safe_d += ~desired_velocity
		 */
	}
	/**
	 * Stop With distance , in these state the vehicle should stop within a designated distance "stopDistance". and should go to complete stop, speed = 0
	 */
	else if(CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE || CurrBehavior.state == STOPPING_STATE)
	{
		if(CurrBehavior.stopDistance > 0)
		{
			desired_acceleration = (-CurrStatus.speed*CurrStatus.speed)/(2.0*CurrBehavior.stopDistance);
			desired_velocity = CurrStatus.speed + (desired_acceleration * dt);
		}
		else
		{
			desired_velocity = 0;
			desired_acceleration = m_VehicleInfo.max_deceleration;
		}
	}
	/**
	 * No other supported state, vehicle should stop if unknown state is sent
	 */
	else
	{
		desired_velocity = 0;
		desired_acceleration = m_VehicleInfo.max_deceleration;
	}

	/**
	 * Never target velocity higher than the one assigned to OpenPlanner in op_common_params
	 */
	if(desired_velocity > m_VehicleInfo.max_speed_forward)
	{
		desired_velocity = m_VehicleInfo.max_speed_forward;
	}
	else if(desired_velocity < 0)
	{
		std::cout << "Desired velocity shouldn't be Zero !! check op_controller !!!! " << std::endl;
		desired_velocity = 0;
	}

	vel_d = desired_velocity;
	acc_d = desired_acceleration;
}

int ACC::UpdateVelocity(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVel, PlannerHNS::SHIFT_POS& desiredShift)
{

	double e_d = 0, e_v = 0;
	double desired_velocity = 0, desired_acceleration = 0, desired_distance = 0, safe_follow_distance = 0;

	CalculateVelocityDesired(dt, CurrStatus, CurrBehavior, desired_velocity, desired_acceleration, desired_distance, safe_follow_distance);

	e_d = (desired_distance - safe_follow_distance); //Follow distance error
	e_v = (desired_velocity - CurrStatus.speed); //Target max velocity error

	/**
	 * Reset PID controller of switching from follow state to any other state other vice versa.
	 */
	if(CurrBehavior.state != m_PrevBehaviorStatus.state && (CurrBehavior.state == FOLLOW_STATE || m_PrevBehaviorStatus.state == FOLLOW_STATE))
	{
		m_pidFollow.Reset();
		m_pidAccel.Reset();
		m_pidBrake.Reset();
	}

	double accel_d = 0, accel_v = 0, brake_v = 0;
	if(desired_acceleration >= m_Params.avg_engine_brake_accel) //accelerate , cruise, small decelerate
	{
		m_pidBrake.Reset();
		accel_v = m_pidAccel.getTimeDependentPID(e_v, dt);
		accel_d = m_pidFollow.getTimeDependentPID(e_d, dt);
		brake_v = 0;
	}
	else  // braking
	{
		m_pidAccel.Reset();
		m_pidFollow.Reset();
		if(CurrBehavior.state == FOLLOW_STATE)
		{
			brake_v = m_pidBrake.getTimeDependentPID(-e_d, dt);
		}
		else
		{
			brake_v = m_pidBrake.getTimeDependentPID(-e_v, dt);
		}
		desiredVel = 0;
	}

	/**
	 * In case of follow state, use the minimum from follow PID and Accel PID. to be able to not exceed the max target velocity
	 */
	if(CurrBehavior.state == FOLLOW_STATE)
	{
		if(accel_d < accel_v)
		{
			desiredVel = accel_d;
		}
		else
		{
			desiredVel = accel_v;
		}
	}
	else
	{
		desiredVel = accel_v;
	}

//	desiredBrake = brake_v;
	desiredShift = PlannerHNS::SHIFT_POS_DD;
	return 1;
}

void  ACC::UpdateVelocityModelBased(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVel, PlannerHNS::SHIFT_POS& desiredShift)
{
	bool bSwitch = false;

	if(CurrBehavior.state != m_PrevBehaviorStatus.state)
	{
		bSwitch = true;
	}

	if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{
		double acceleration_critical = m_VehicleInfo.max_acceleration * m_Params.accelPushRatio;

		if(CurrBehavior.maxVelocity < CurrStatus.speed)
		{
			acceleration_critical = m_VehicleInfo.max_deceleration * m_Params.brakePushRatio;
		}

		double incr_vel = acceleration_critical * dt;

		if(CurrStatus.speed < 1.0 && CurrBehavior.maxVelocity > 1.0)
		{
			incr_vel += 1.0;
		}

		desiredVel = incr_vel + CurrStatus.speed;

		//std::cout << "Forward: max_velocity: " <<  CurrBehavior.maxVelocity << ", currSpeed: " << CurrStatus.speed << ", desiredVel: " << desiredVel << ", acceleration: " << acceleration_critical << std::endl;

	}
	else if(CurrBehavior.state == STOPPING_STATE || CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE)
	{
		double deceleration_critical = m_VehicleInfo.max_deceleration;
		double distance_to_stop = CurrBehavior.stopDistance ;
		if(distance_to_stop != 0)
		{
			deceleration_critical = (-CurrStatus.speed*CurrStatus.speed)/(2.0*distance_to_stop);
		}

		deceleration_critical = deceleration_critical * m_Params.brakePushRatio;

		desiredVel = (deceleration_critical * dt) + CurrStatus.speed;

		std::cout<<"distance : "<<CurrBehavior.stopDistance<<std::endl;
		std::cout<<"desried velocity : "<<desiredVel<<std::endl<<std::endl;
		//std::cout << "STOP: stop_distance: " <<  distance_to_stop << ", desiredVel: " << desiredVel << ", Deceleration: " << deceleration_critical << ", dt: " << dt << std::endl;
	}
	else if(CurrBehavior.state == FOLLOW_STATE)
	{
		double crash_d = CurrBehavior.followDistance;
		double safe_d = CurrBehavior.stopDistance;
		double min_follow_distance = m_Params.min_safe_follow_distance*2.0 + CurrStatus.speed;
		double diff = crash_d - safe_d;
		double target_a = 0;

		/**
		 * Following Conditions
		 */
		if(diff < m_Params.min_safe_follow_distance*2.0)
		{
			double brake_distance = crash_d - m_Params.min_safe_follow_distance*2.0;

			if(brake_distance > 0)
			{
				target_a = (-CurrStatus.speed*CurrStatus.speed)/(2.0*brake_distance);
			}
			else
			{
				target_a = -9.8*4; //stop with -4G
			}
		}
		else if(diff > (m_Params.min_safe_follow_distance*2.0 + CurrStatus.speed))
		{
			target_a = m_VehicleInfo.max_acceleration;
		}

		/**
		 * When in a curve , should driver slower then followed vehicle
		 */
		if(CurrStatus.speed > CurrBehavior.maxVelocity && target_a > m_VehicleInfo.max_deceleration)
		{

			target_a = m_VehicleInfo.max_deceleration;
		}

		/**
		 * Apply acceleration push factors
		 */
		if(target_a > 0)
		{
			target_a = target_a * m_Params.accelPushRatio;
		}
		else
		{
			target_a = target_a * m_Params.brakePushRatio;
		}

		desiredVel = (target_a * dt) + CurrStatus.speed;

		if(CurrStatus.speed < 1.0 && CurrBehavior.maxVelocity > 1.0 && desiredVel > 0)
		{
			desiredVel += 1.0;
		}

		//std::cout << "Follow: safe_d: " <<  safe_d << ", follow_d: " << crash_d << ", diff: " << diff << ", accel-decel: " << target_a << ", desiredVel: " << desiredVel << std::endl;
	}
	else
	{
		desiredVel = 0;
	}

	if(desiredVel >  m_VehicleInfo.max_speed_forward)
	{
		desiredVel = m_VehicleInfo.max_speed_forward;
	}
	else if(desiredVel < 0)
	{
		desiredVel = 0;
	}
}

PlannerHNS::VehicleState ACC::DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior, const PlannerHNS::VehicleState& vehicleState)
{
	PlannerHNS::VehicleState desiredState;

	if(m_bModelBased)
	{
		UpdateVelocityModelBased(dt, vehicleState, behavior, desiredState.speed, desiredState.shift);
	}
	else
	{
		UpdateVelocity(dt, vehicleState, behavior, desiredState.speed, desiredState.shift);
	}

	m_PrevBehaviorStatus = behavior;
	m_PrevDesiredState = desiredState;

	bool bEnoughEvidence = CalcAvgRelativeSpeedFromDistance(dt, behavior, m_AverageRelativeSpeed);

	return desiredState;
}

bool ACC::CalcAvgRelativeSpeedFromDistance(const double& dt, const PlannerHNS::BehaviorState& CurrBehavior, double avg_relative_speed)
{
	bool bEnoughEvidence = false;
	if(CurrBehavior.state == FOLLOW_STATE)
	{
		if(m_RelativeSpeeds.size() > 0)
		{
			double relative_instant_velocity = (CurrBehavior.followDistance - m_PrevFollowDistance)/dt;
			m_RelativeSpeeds.push_back(relative_instant_velocity);

			if(m_RelativeSpeeds.size() > 10)
			{
				double vel_sum = 0;
				for(auto& v: m_RelativeSpeeds)
				{
					vel_sum += v;
				}
				avg_relative_speed = vel_sum / m_RelativeSpeeds.size();
				m_RelativeSpeeds.erase(m_RelativeSpeeds.begin()+0);
				bEnoughEvidence = true;
			}
		}
		else
		{
			m_RelativeSpeeds.push_back(0);
		}

		m_PrevFollowDistance = CurrBehavior.followDistance;
	}
	else
	{
		m_PrevFollowDistance = 0;
		m_RelativeSpeeds.clear();
	}

	return bEnoughEvidence;
}

} /* namespace SimulationNS */
