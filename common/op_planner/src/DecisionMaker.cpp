
/// \file DecisionMaker.cpp
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016

#include "op_planner/DecisionMaker.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"

namespace PlannerHNS
{

	DecisionMaker::DecisionMaker()
	{
		m_CurrGlobalId = -1;
		m_iCurrentTotalPathId = 0;
		//pLane = nullptr;
		m_pCurrentBehaviorState = nullptr;
		m_pGoToGoalState = nullptr;
		m_pWaitState = nullptr;
		m_pMissionCompleteState = nullptr;
		m_pAvoidObstacleState = nullptr;
		m_pTrafficLightStopState = nullptr;
		m_pTrafficLightWaitState = nullptr;
		m_pStopSignStopState = nullptr;
		m_pStopSignWaitState = nullptr;
		m_pFollowState = nullptr;
		m_pMissionCompleteState = nullptr;
		m_pGoalState = nullptr;
		m_pGoToGoalState = nullptr;
		m_pWaitState = nullptr;
		m_pInitState = nullptr;
		m_pFollowState = nullptr;
		m_pAvoidObstacleState = nullptr;
		m_pStopState = nullptr;
		m_bRequestNewGlobalPlan = false;
		m_bUseInternalACC = false;
	}

	DecisionMaker::~DecisionMaker()
	{
		delete m_pStopState;
		delete m_pMissionCompleteState;
		delete m_pGoalState;
		delete m_pGoToGoalState;
		delete m_pWaitState;
		delete m_pInitState;
		delete m_pFollowState;
		delete m_pAvoidObstacleState;
		delete m_pTrafficLightStopState;
		delete m_pTrafficLightWaitState;
		delete m_pStopSignWaitState;
		delete m_pStopSignStopState;
	}

	void DecisionMaker::Init(const ControllerParams &ctrlParams, const PlannerHNS::PlanningParams &params, const CAR_BASIC_INFO &carInfo)
	{
		m_CarInfo = carInfo;
		m_ControlParams = ctrlParams;
		m_params = params;
		m_original_params = params;

		m_VelocityController.Init(m_ControlParams, m_CarInfo, true);

		m_pidVelocity.Init(0.01, 0.004, 0.01);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.005, 0.005, 0.01);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		m_pidFollowing.Init(0.05, 0.05, 0.01);
		m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

		InitBehaviorStates();

		if (m_pCurrentBehaviorState)
		{
			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
			PreCalculatedConditions *pValues = m_pCurrentBehaviorState->GetCalcParams();
			pValues->minStoppingDistance = m_params.horizonDistance;
			pValues->iCentralTrajectory = m_params.rollOutNumber / 2;
			pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
			pValues->iCurrSafeLane = 0;
			pValues->stoppingDistances.clear();
			pValues->stoppingDistances.push_back(m_params.horizonDistance);
			pValues->currentVelocity = 0;
			pValues->bTrafficIsRed = false;
			pValues->currentTrafficLightID = -1;
			pValues->currentStopSignID = -1;
			pValues->bFullyBlock = false;
			pValues->bFinalLocalTrajectory = false;
			pValues->distanceToNext = m_params.horizonDistance;
			pValues->velocityOfNext = 0;
			pValues->distanceToGoal = m_params.horizonDistance;
			pValues->currentGoalID = -1;
			pValues->prevGoalID = -1;
		}
	}

	void DecisionMaker::UpdateAvoidanceParams(bool enable_swerve, int roll_out_numbers)
	{
		if (enable_swerve == false && enable_swerve != m_params.enableSwerving)
		{
			m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
		}

		m_params.enableSwerving = enable_swerve;
		m_params.rollOutNumber = roll_out_numbers;
	}

	void DecisionMaker::InitBehaviorStates()
	{

		m_pStopState = new StopStateII(&m_params, 0, 0);
		m_pMissionCompleteState = new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
		m_pGoalState = new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
		m_pGoToGoalState = new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
		m_pInitState = new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

		m_pFollowState = new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pAvoidObstacleState = new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pStopSignWaitState = new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pStopSignStopState = new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

		m_pTrafficLightWaitState = new TrafficLightWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pTrafficLightStopState = new TrafficLightStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

		m_pStopState->InsertNextState(m_pGoToGoalState);
		m_pStopState->InsertNextState(m_pGoalState);
		m_pStopState->decisionMakingCount = 0;

		m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
		m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
		m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
		m_pGoToGoalState->InsertNextState(m_pFollowState);
		m_pGoToGoalState->InsertNextState(m_pStopState);
		m_pGoToGoalState->decisionMakingCount = 0; //m_params.nReliableCount;

		m_pGoalState->InsertNextState(m_pGoToGoalState);

		m_pStopSignWaitState->decisionMakingTime = m_params.stopSignStopTime;
		m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
		m_pStopSignWaitState->InsertNextState(m_pGoalState);

		m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);

		m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
		m_pTrafficLightWaitState->InsertNextState(m_pGoalState);

		m_pFollowState->InsertNextState(m_pAvoidObstacleState);
		m_pFollowState->InsertNextState(m_pStopSignStopState);
		m_pFollowState->InsertNextState(m_pTrafficLightStopState);
		m_pFollowState->InsertNextState(m_pGoalState);
		m_pFollowState->decisionMakingCount = 0; //m_params.nReliableCount;

		m_pInitState->decisionMakingCount = 0; //m_params.nReliableCount;

		m_pCurrentBehaviorState = m_pInitState;
	}

	bool DecisionMaker::GetNextTrafficLight(const int &prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight> &trafficLights, PlannerHNS::TrafficLight &trafficL)
	{
		for (unsigned int i = 0; i < trafficLights.size(); i++)
		{
			double d = hypot(trafficLights.at(i).pose.pos.y - state.pos.y, trafficLights.at(i).pose.pos.x - state.pos.x);
			if (d <= trafficLights.at(i).stoppingDistance)
			{
				double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pose.pos.a), UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

				if (a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
				{
					//std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
					trafficL = trafficLights.at(i);
					return true;
				}
			}
		}

		return false;
	}

	void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState &car_state,
																	 const bool &bEmergencyStop, const std::vector<TrafficLight> &detectedLights,
																	 const TrajectoryCost &bestTrajectory)
	{
		if (m_TotalPaths.size() == 0)
			return;

		PreCalculatedConditions *pValues = m_pCurrentBehaviorState->GetCalcParams();

		if (m_CarInfo.max_deceleration != 0)
			pValues->minStoppingDistance = -pow(car_state.speed, 2) / (2.0 * m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
		else
			pValues->minStoppingDistance = m_params.horizonDistance;

		pValues->stoppingDistances.clear();
		pValues->stoppingDistances.push_back(m_params.horizonDistance);
		pValues->stoppingDistances.push_back(pValues->minStoppingDistance);
		pValues->currentVelocity = car_state.speed;
		pValues->bTrafficIsRed = false;
		pValues->currentTrafficLightID = -1;
		pValues->bFullyBlock = false;
		pValues->bFinalLocalTrajectory = false;

		pValues->distanceToNext = bestTrajectory.closest_obj_distance;
		pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

		/**
	 * Global Lanes section, set global path index and ID
	 */
		//if(bestTrajectory.lane_index >= 0 && m_bRequestNewGlobalPlan == false)
		if (bestTrajectory.lane_index >= 0 && bestTrajectory.lane_index < m_TotalPaths.size())
		{
			pValues->iCurrSafeLane = bestTrajectory.lane_index;
		}
		else
		{
			PlannerHNS::RelativeInfo info;
			PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPaths, state, m_params.rollOutDensity * m_params.rollOutNumber / 2.0 + 0.1, info);
			pValues->iCurrSafeLane = info.iGlobalPath;
		}

		m_iCurrentTotalPathId = pValues->iCurrSafeLane;

		for (unsigned int ig = 0; ig < m_TotalPaths.size(); ig++)
		{
			if (ig == m_iCurrentTotalPathId && m_TotalPaths.at(ig).size() > 0)
			{
				m_CurrGlobalId = m_TotalPaths.at(ig).at(0).gid;
			}
		}

		/**
	 * ---------------------------------------------------------------------
	 */

		/**
	 * Local Trajectory section, set local trajectory index
	 */

		if (m_LanesRollOuts.at(m_iCurrentTotalPathId).size() <= 1)
		{
			m_params.rollOutNumber = 0;
			m_params.enableSwerving = false;
		}
		else
		{
			m_params.rollOutNumber = m_LanesRollOuts.at(m_iCurrentTotalPathId).size() - 1;
			m_params.enableSwerving = m_original_params.enableSwerving;
		}

		pValues->iCentralTrajectory = m_pCurrentBehaviorState->m_pParams->rollOutNumber / 2;

		if (pValues->iPrevSafeTrajectory < 0)
		{
			pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;
		}

		if (bestTrajectory.index >= 0 && bestTrajectory.index < (int)m_LanesRollOuts.at(m_iCurrentTotalPathId).size())
		{
			pValues->iCurrSafeTrajectory = bestTrajectory.index;
		}
		else
		{
			pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;
		}
		/**
 	 * --------------------------------------------------------------------
 	 */

		pValues->bFullyBlock = bestTrajectory.bBlocked;

		//double critical_long_front_distance = m_params.additionalBrakingDistance + m_params.verticalSafetyDistance;
		double critical_long_front_distance = m_params.verticalSafetyDistance + m_CarInfo.length;

		pValues->distanceToGoal = PlannerHNS::PlanningHelpers::GetDistanceFromPoseToEnd(state, m_TotalPaths.at(pValues->iCurrSafeLane));
		if ((pValues->distanceToGoal < -m_params.goalDiscoveryDistance) || (pValues->distanceToGoal > m_params.horizonDistance))
		{
			pValues->distanceToGoal = m_params.horizonDistance;
		}

		pValues->stoppingDistances.push_back(pValues->distanceToGoal);

		//if(ReachEndOfGlobalPath(pValues->minStoppingDistance + critical_long_front_distance, pValues->iCurrSafeLane)) //deprecated 27-August-2018
		if (pValues->distanceToGoal < m_params.goalDiscoveryDistance)
		{
			pValues->currentGoalID = -1;
		}
		else
		{
			pValues->currentGoalID = 1;
			pValues->prevGoalID = 1;
		}

		pValues->stoppingDistances.push_back(pValues->distanceToGoal);

		int stopLineID = -1;
		int stopSignID = -1;
		int trafficLightID = -1;
		double distanceToClosestStopLine = 0;
		bool bGreenTrafficLight = true;

		distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPaths.at(pValues->iCurrSafeLane), state, m_params.giveUpDistance, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;
		if (distanceToClosestStopLine > m_params.giveUpDistance && distanceToClosestStopLine < (pValues->minStoppingDistance + 2.0))
		{
			if (m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
			{
				pValues->currentTrafficLightID = trafficLightID;
				//std::cout << "Detected Traffic Light: " << trafficLightID << std::endl;
				for (unsigned int i = 0; i < detectedLights.size(); i++)
				{
					if (detectedLights.at(i).id == trafficLightID)
						bGreenTrafficLight = (detectedLights.at(i).lightType == GREEN_LIGHT);
				}
			}

			if (m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior || m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
			{
				pValues->currentStopSignID = stopSignID;
				pValues->stoppingDistances.push_back(distanceToClosestStopLine);
				//std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << std::endl;
			}
		}

		//std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

		pValues->bTrafficIsRed = !bGreenTrafficLight;

		if (bEmergencyStop)
		{
			pValues->bFullyBlock = true;
			pValues->distanceToNext = 1;
			pValues->velocityOfNext = 0;
		}

		if (m_Path.size() > 0 && m_TotalOriginalPaths.size() > 0)
		{
			double d_between_ends = hypot(m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.y - m_Path.back().pos.y, m_TotalOriginalPaths.at(m_iCurrentTotalPathId).back().pos.x - m_Path.back().pos.x);
			if (d_between_ends < m_params.pathDensity)
			{
				pValues->bFinalLocalTrajectory = true;
			}
		}

		if (m_TotalPaths.size() > 1)
		{
			for (unsigned int i = 0; i < m_TotalPaths.size(); i++)
			{
				RelativeInfo curr_total_path_inf;
				int dummy_index = 0;
				PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(i), state, curr_total_path_inf, dummy_index);
				pValues->distanceToChangeLane = m_TotalPaths.at(i).back().cost - curr_total_path_inf.perp_point.cost;
				if (pValues->distanceToChangeLane < m_params.microPlanDistance * 0.8)
				{
					m_bRequestNewGlobalPlan = true;
				}
			}
		}

		// 	if(m_RollOuts.size() > 2)
		// 	{
		// 		std::cout << "From Decision Maker, RollIndex: " << bestTrajectory.index << ", SafeTraj: " << pValues->iCurrSafeTrajectory << ", PrevTraj: " <<pValues->iPrevSafeTrajectory << ", Blocked: " << bestTrajectory.bBlocked
		// 		<< ", dtoNext:" <<  pValues->distanceToNext << ", dtoAvoid: " << m_params.minDistanceToAvoid << std::endl;
		// 	}
	}

	bool DecisionMaker::ReachEndOfGlobalPath(const double &min_distance, const int &iGlobalPathIndex)
	{
		if (m_TotalPaths.size() == 0)
			return false;

		PlannerHNS::RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(iGlobalPathIndex), state, info);

		double d = 0;
		for (unsigned int i = info.iFront; i < m_TotalPaths.at(iGlobalPathIndex).size() - 1; i++)
		{
			d += hypot(m_TotalPaths.at(iGlobalPathIndex).at(i + 1).pos.y - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.y, m_TotalPaths.at(iGlobalPathIndex).at(i + 1).pos.x - m_TotalPaths.at(iGlobalPathIndex).at(i).pos.x);
			if (d > min_distance)
				return false;
		}

		return true;
	}

	void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint>> &globalPath)
	{
		if (m_pCurrentBehaviorState)
		{
			m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
			m_bRequestNewGlobalPlan = false;
			m_TotalOriginalPaths = globalPath;
			m_prev_index.clear();
			for (unsigned int i = 0; i < globalPath.size(); i++)
			{
				m_prev_index.push_back(0);
			}
		}
	}

	bool DecisionMaker::SelectSafeTrajectory()
	{
		bool bNewTrajectory = false;
		PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

		if (!preCalcPrams || m_LanesRollOuts.at(m_iCurrentTotalPathId).size() == 0)
			return bNewTrajectory;

		int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
		int index_limit = m_Path.size() / 2.0 + 1;

		if ((currIndex > index_limit || preCalcPrams->bRePlan || preCalcPrams->bNewGlobalPath) && !preCalcPrams->bFinalLocalTrajectory)
		{
			//std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalPath.at(0).size() << ", PrevLocal: " << m_Path.size();

			m_Path = m_LanesRollOuts.at(m_iCurrentTotalPathId).at(preCalcPrams->iCurrSafeTrajectory);
			//std::cout << ", NewLocal: " << m_Path.size() << std::endl;

			preCalcPrams->bNewGlobalPath = false;
			preCalcPrams->bRePlan = false;
			bNewTrajectory = true;
		}

		return bNewTrajectory;
	}

	PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState &vehicleState)
	{
		PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

		m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
		if (m_pCurrentBehaviorState == 0)
			m_pCurrentBehaviorState = m_pInitState;

		PlannerHNS::BehaviorState currentBehavior;

		currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
		currentBehavior.followDistance = preCalcPrams->distanceToNext;

		currentBehavior.minVelocity = 0;
		currentBehavior.stopDistance = preCalcPrams->distanceToStop();
		currentBehavior.followVelocity = preCalcPrams->velocityOfNext;
		if (preCalcPrams->iPrevSafeTrajectory < 0 || preCalcPrams->iPrevSafeTrajectory >= m_LanesRollOuts.at(m_iCurrentTotalPathId).size())
		{
			currentBehavior.iTrajectory = preCalcPrams->iCurrSafeTrajectory;
		}
		else
		{
			currentBehavior.iTrajectory = preCalcPrams->iPrevSafeTrajectory;
		}

		currentBehavior.iLane = m_iCurrentTotalPathId;

		//double average_braking_distance = -pow(vehicleState.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance; // average_braking_distance replaced by minStoppingDistance on 28th August 2018
		double indication_distance = preCalcPrams->minStoppingDistance;
		if (indication_distance < m_params.minIndicationDistance)
		{
			indication_distance = m_params.minIndicationDistance;
		}

		currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, indication_distance);
		if (currentBehavior.state == GOAL_STATE || currentBehavior.state == FINISH_STATE || m_params.maxSpeed == 0)
		{
			currentBehavior.indicator = INDICATOR_BOTH;
		}

		return currentBehavior;
	}

	double DecisionMaker::UpdateVelocityDirectlyToTrajectorySmooth(BehaviorState &beh, const VehicleState &CurrStatus, const double &dt)
	{

		PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

		if (!preCalcPrams || m_TotalPaths.size() == 0)
			return 0;

		BehaviorState beh_with_max = beh;

		RelativeInfo total_info;
		PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
		beh_with_max.maxVelocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, preCalcPrams->minStoppingDistance * m_ControlParams.curveSlowDownRatio);
		if (beh_with_max.maxVelocity > m_params.maxSpeed)
		{
			beh_with_max.maxVelocity = m_params.maxSpeed;
		}
		//VehicleState desired_state =  m_VelocityController.DoOneStep(dt, beh_with_max, CurrStatus);
		double max_velocity = beh_with_max.maxVelocity;
		if(preCalcPrams->bFinalLocalTrajectory)
		{
			max_velocity = max_velocity * 0.9; //slow down for stop
		}
		double desired_velocity = 0.0;

#if 1
		if (beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE)
		{
			double velocity_diff = beh_with_max.maxVelocity - CurrStatus.speed;
			double adjust_velocity = 0.0;

			if (velocity_diff > 0.0)
				adjust_velocity = std::min(velocity_diff * 0.1, 0.5);
			else
				adjust_velocity = std::max(velocity_diff * 0.1, -0.5);

			desired_velocity = CurrStatus.speed + adjust_velocity;

			if (preCalcPrams->iCurrSafeTrajectory != preCalcPrams->iCentralTrajectory)
			{
				std::min(max_velocity * 0.7, desired_velocity);
			}
		}

		else if (beh.state == FOLLOW_STATE)
		{
			double distance_to_stop = beh.followDistance - (3.0 * m_CarInfo.length);
			double distance_desired_velocity = 0.8 * (distance_to_stop * 0.14);
			double current_desired_velocity = 0.2 * CurrStatus.speed;
			double final_desired_velocity = distance_desired_velocity + current_desired_velocity;

			double velocity_diff = (final_desired_velocity - CurrStatus.speed);
			double adjust_velocity = 0.0;

			if (velocity_diff > 0.0)
				adjust_velocity = std::min(velocity_diff * 0.1, 0.5);
			else
				adjust_velocity = std::max(velocity_diff * 0.27, -1.25);

			desired_velocity = CurrStatus.speed + adjust_velocity;

			if (CurrStatus.speed < desired_velocity)
			{
				desired_velocity = (CurrStatus.speed + desired_velocity) * 0.5;
			}

			if(distance_to_stop < 0.1 || desired_velocity < 0.2)
				desired_velocity = 0.0;
			
			std::cout<<"[follow state] distance to stop : "<<distance_to_stop<<std::endl;
			std::cout<<"[follow state] final velocity : "<<final_desired_velocity<<std::endl<<std::endl;
		}

		else if (beh.state == STOPPING_STATE || beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
		{
			double target_deceleration;
			//double f_distanceRateVelocity = 0.9*(-1.0 * sqrt(0.000001 * pow(beh.stopDistance, 3)) + (0.005 * pow(beh.stopDistance, 2)));
			//double f_distanceRateVelocity = 0.8 * sqrt(0.0001 * pow((beh.stopDistance), 3)) - (0.0001 * pow((beh.stopDistance), 2));
			double distance_desired_velocity = 0.8 * (beh.stopDistance * 0.14); // weight1 * (distance / time)
			double current_desired_velocity = 0.2 * CurrStatus.speed;			// weight2 * currentVelocity
			double m_newDesiredVelocity = distance_desired_velocity + current_desired_velocity;

			double velocity_diff = m_newDesiredVelocity - CurrStatus.speed;
			double adjust_velocity = 0.0;

			if (velocity_diff > 0.0)
				adjust_velocity = std::min(velocity_diff * 0.1, 0.5);
			else
				adjust_velocity = std::max(velocity_diff * 0.27, -1.25);

			desired_velocity = adjust_velocity + CurrStatus.speed;

			if (CurrStatus.speed < desired_velocity)
			{
				desired_velocity = (CurrStatus.speed + desired_velocity) * 0.5;
			}

			if(beh.stopDistance < 0.1)
				desired_velocity = 0.0;

			std::cout<<"[stop state] beh.stopDistance : " <<beh.stopDistance<<std::endl;
			std::cout<<"[stop state] desire velocity : "<<desired_velocity<<std::endl;
		}

		else if (beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
		{
			desired_velocity = 0;
		}

		else
		{
			desired_velocity = 0;
		}

		if (desired_velocity > m_params.maxSpeed)
		{
			desired_velocity = m_params.maxSpeed;
		}

		else if (desired_velocity < 0)
		{
			desired_velocity = 0;
		}
#endif

		for (unsigned int i = 0; i < m_Path.size(); i++)
		{
			m_Path.at(i).v = desired_velocity;
		}

		if(m_lanechange_deceleration && (desired_velocity > 5.5))
		{
			desired_velocity = 5.5;
			// std::cout << "Deceleration. desired_velocity : " << desired_velocity << std::endl;
		} // woocheol
		
		return desired_velocity;
	}

	double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState &beh, const VehicleState &CurrStatus, const double &dt)
	{

		PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

		if (!preCalcPrams || m_TotalPaths.size() == 0)
			return 0;

		RelativeInfo info, total_info;
		PlanningHelpers::GetRelativeInfo(m_TotalPaths.at(m_iCurrentTotalPathId), state, total_info);
		PlanningHelpers::GetRelativeInfo(m_Path, state, info);
		double max_velocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalPaths.at(m_iCurrentTotalPathId), total_info, total_info.iBack, preCalcPrams->minStoppingDistance);
		if (max_velocity > m_params.maxSpeed)
		{
			max_velocity = m_params.maxSpeed;
		}

		//std::cout << "Max Velocity : " << max_velocity << "," << m_params.maxSpeed << std::endl;

		double critical_long_front_distance = m_CarInfo.length / 2.0;
		double desiredVelocity = 0;

		if (beh.state == STOPPING_STATE || beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
		{
			double deceleration_critical = 0;
			double distance_to_stop = beh.stopDistance;
			if (distance_to_stop != 0)
				deceleration_critical = (-CurrStatus.speed * CurrStatus.speed) / (2.0 * distance_to_stop);

			if (deceleration_critical >= 0)
				deceleration_critical = m_CarInfo.max_deceleration;

			desiredVelocity = deceleration_critical * dt + CurrStatus.speed;

			desiredVelocity = 0; //for CARLA

			//std::cout << "Stopping : V: " << CurrStatus.speed << ", A: " << deceleration_critical << ", dt: " << dt << std::endl;
			//std::cout << "Stopping (beh, brake): (" << beh.stopDistance << ", " << preCalcPrams->minStoppingDistance << ") , desiredPID=" << desiredVelocity << ", To Goal: " << preCalcPrams->distanceToGoal <<  std::endl;
		}
		else if (beh.state == FOLLOW_STATE)
		{

			double deceleration_critical = 0;
			double distance_to_stop = beh.followDistance - critical_long_front_distance - m_params.additionalBrakingDistance;
			double sudden_stop_distance = -pow((CurrStatus.speed - beh.followVelocity), 2) / m_CarInfo.max_deceleration;

			if (distance_to_stop != 0)
				deceleration_critical = (-CurrStatus.speed * CurrStatus.speed) / (2.0 * distance_to_stop);

			if (deceleration_critical >= 0)
				deceleration_critical = m_CarInfo.max_deceleration;

			desiredVelocity = deceleration_critical * dt + CurrStatus.speed;

			//		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
			//		{
			//			desiredVelocity  = desiredVelocity * 0.75;
			//		}

			if (beh.followVelocity > CurrStatus.speed)
				desiredVelocity = CurrStatus.speed;

			desiredVelocity = 0; // for CARLA

			//std::cout << "Following V: " << CurrStatus.speed << ", Desired V: " << beh.followVelocity << ", A: " << deceleration_critical << ", d_to_stop: " << distance_to_stop << ", sudden_stop_d" << sudden_stop_distance << std::endl;
			//std::cout << "Desired Vel: " << desiredVelocity << std::endl;
		}
		else if (beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE)
		{

			double acceleration_critical = m_CarInfo.max_acceleration;

			if (max_velocity < CurrStatus.speed)
				acceleration_critical = m_CarInfo.max_deceleration;

			desiredVelocity = (acceleration_critical * dt) + CurrStatus.speed;

			//For CARLA
			//		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
			//		{
			//			desiredVelocity  = max_velocity * 0.75;
			//		}
			//		else
			desiredVelocity = max_velocity;

			//std::cout << "bEnd : " << preCalcPrams->bFinalLocalTrajectory << ", Min D: " << preCalcPrams->minStoppingDistance << ", D To Goal: " << preCalcPrams->distanceToGoal << std::endl;
			//std::cout << "Forward: dt" << dt << " ,Target vel: " << desiredVelocity << ", Acc: " << acceleration_critical << ", Max Vel: " << max_velocity << ", Curr Vel: " << CurrStatus.speed << ", break_d: " << m_params.additionalBrakingDistance  << std::endl;
			//std::cout << "Forward Target Acc: " << acceleration_critical  << ", PID Velocity: " << desiredVelocity << ", Max Velocity : " << max_velocity  << std::endl;
		}
		else if (beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
		{
			desiredVelocity = 0;
		}
		else
		{
			desiredVelocity = 0;
		}

		if (desiredVelocity > m_params.maxSpeed)
		{
			desiredVelocity = m_params.maxSpeed;
		}
		else if (desiredVelocity < 0)
		{
			desiredVelocity = 0;
		}

		for (unsigned int i = 0; i < m_Path.size(); i++)
		{
			m_Path.at(i).v = desiredVelocity;
		}

		return max_velocity;
	}

	PlannerHNS::BehaviorState DecisionMaker::DoOneStep(
		const double &dt,
		const PlannerHNS::WayPoint currPose,
		const PlannerHNS::VehicleState &vehicleState,
		const std::vector<TrafficLight> &trafficLight,
		const TrajectoryCost &tc,
		const bool &bEmergencyStop)
	{
		PlannerHNS::BehaviorState beh;
		state = currPose;
		m_TotalPaths.clear();

		if (m_prev_index.size() != m_TotalOriginalPaths.size())
		{
			m_prev_index.clear();
			for (unsigned int i = 0; i < m_TotalOriginalPaths.size(); i++)
			{
				m_prev_index.push_back(0);
			}
		}

		for (unsigned int i = 0; i < m_TotalOriginalPaths.size(); i++)
		{
			t_centerTrajectorySmoothed.clear();
			m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPaths.at(i), state, m_params.horizonDistance, m_params.pathDensity, t_centerTrajectorySmoothed, m_prev_index.at(i));

			if (m_prev_index.at(i) > 0)
				m_prev_index.at(i) = m_prev_index.at(i) - 1;

			m_TotalPaths.push_back(t_centerTrajectorySmoothed);
		}

		if (m_TotalPaths.size() == 0)
			return beh;

		//UpdateCurrentLane(m_params.maxLaneSearchDistance);

		CalculateImportantParameterForDecisionMaking(vehicleState, bEmergencyStop, trafficLight, tc);

		beh = GenerateBehaviorState(vehicleState);

		beh.bNewPlan = SelectSafeTrajectory();

		if (m_bUseInternalACC)
		{
			beh.maxVelocity = UpdateVelocityDirectlyToTrajectorySmooth(beh, vehicleState, dt);
			// std::cout << "ACC" << std::endl;
		}
		else
		{
			beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);
			// std::cout << "NOT ACC" << std::endl;
		}

		//std::cout << "Evaluated Rollouts size: " << m_LanesRollOuts.size() << std::endl;
		//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

		return beh;
	}

} /* namespace PlannerHNS */
