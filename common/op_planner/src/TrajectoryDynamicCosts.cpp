
/// \file TrajectoryDynamicCosts.cpp
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+
/// \author Hatem Darweesh
/// \date Jan 14, 2018

#include "op_planner/TrajectoryDynamicCosts.h"
#include "op_planner/MatrixOperations.h"
#include "float.h"

namespace PlannerHNS
{


TrajectoryDynamicCosts::TrajectoryDynamicCosts()
{
	m_PrevCostIndex = -1;
	m_WeightLong = 1.0;
	m_WeightLat = 1.2;
	m_LateralSkipDistance = 50;
	m_PrevIndex = -1;
	m_WeightPriority = 0.9;
	m_WeightTransition = 0.9;
}

TrajectoryDynamicCosts::~TrajectoryDynamicCosts()
{
}

TrajectoryCost TrajectoryDynamicCosts::DoOneStepStatic(const vector<vector<WayPoint> >& rollOuts,
		const vector<WayPoint>& totalPaths, const WayPoint& currState,
		const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState,
		const std::vector<PlannerHNS::DetectedObject>& obj_list, const int& iCurrentIndex)
{
	TrajectoryCost bestTrajectory;
	bestTrajectory.bBlocked = true;
	bestTrajectory.closest_obj_distance = params.horizonDistance;
	bestTrajectory.closest_obj_velocity = 0;
	bestTrajectory.index = -1;

	RelativeInfo obj_info;
	PlanningHelpers::GetRelativeInfo(totalPaths, currState, obj_info);
	int currIndex = params.rollOutNumber/2 + floor(obj_info.perp_distance/params.rollOutDensity);
	//std::cout <<  "Current Index: " << currIndex << std::endl;
	if(currIndex < 0)
		currIndex = 0;
	else if(currIndex > params.rollOutNumber)
		currIndex = params.rollOutNumber;

	if(m_PrevCostIndex == -1)
		m_PrevCostIndex = params.rollOutNumber/2;

	m_TrajectoryCosts.clear();
	if(rollOuts.size()>0)
	{
		TrajectoryCost tc;
		int centralIndex = params.rollOutNumber/2;
		tc.lane_index = 0;
		for(unsigned int it=0; it< rollOuts.size(); it++)
		{
			tc.index = it;
			tc.relative_index = it - centralIndex;
			tc.distance_from_center = params.rollOutDensity*tc.relative_index;
			tc.priority_cost = fabs(tc.distance_from_center);
			tc.closest_obj_distance = params.horizonDistance;
			if(rollOuts.at(it).size() > 0)
					tc.lane_change_cost = rollOuts.at(it).at(0).laneChangeCost;
			m_TrajectoryCosts.push_back(tc);
		}
	}

	CalculateTransitionCosts(m_TrajectoryCosts, currIndex, params);

	WayPoint p;
	m_AllContourPoints.clear();
	for(unsigned int io=0; io<obj_list.size(); io++)
	{
		for(unsigned int icon=0; icon < obj_list.at(io).contour.size(); icon++)
		{
			p.pos = obj_list.at(io).contour.at(icon);
			p.v = obj_list.at(io).center.v;
			p.id = io;
			p.cost = sqrt(obj_list.at(io).w*obj_list.at(io).w + obj_list.at(io).l*obj_list.at(io).l);
			m_AllContourPoints.push_back(p);
		}
	}

	CalculateLateralAndLongitudinalCostsStatic(m_TrajectoryCosts, rollOuts, totalPaths, currState, m_AllContourPoints, params, carInfo, vehicleState);

	NormalizeCosts(m_TrajectoryCosts);

	int smallestIndex = -1;
	double smallestCost = DBL_MAX;
	double smallestDistance = DBL_MAX;
	double velo_of_next = 0;

	//cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
	for(unsigned int ic = 0; ic < m_TrajectoryCosts.size(); ic++)
	{
		//cout << m_TrajectoryCosts.at(ic).ToString();
		if(!m_TrajectoryCosts.at(ic).bBlocked && m_TrajectoryCosts.at(ic).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(ic).cost;
			smallestIndex = ic;
		}

		if(m_TrajectoryCosts.at(ic).closest_obj_distance < smallestDistance)
		{
			smallestDistance = m_TrajectoryCosts.at(ic).closest_obj_distance;
			velo_of_next = m_TrajectoryCosts.at(ic).closest_obj_velocity;
		}
	}

	if(smallestIndex == -1)
	{
		bestTrajectory.bBlocked = true;
		bestTrajectory.lane_index = 0;
		bestTrajectory.index = m_PrevCostIndex;
		bestTrajectory.closest_obj_distance = smallestDistance;
		bestTrajectory.closest_obj_velocity = velo_of_next;
	}
	else if(smallestIndex >= 0)
	{
		bestTrajectory = m_TrajectoryCosts.at(smallestIndex);
	}

//	cout << "Smallest Index: " << smallestIndex <<", Costs Size: " << m_TrajectoryCosts.size() << ", Rollout Index: " << bestTrajectory.index << endl;
//	cout << "------------------------------------------------------------- " << endl;

	m_PrevIndex = currIndex;
	return bestTrajectory;
}

void TrajectoryDynamicCosts::CalculateLateralAndLongitudinalCostsStatic(vector<TrajectoryCost>& trajectoryCosts,
		const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths,
		const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params,
		const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState)
{
	double critical_lateral_distance =  carInfo.width/2.0 + params.horizontalSafetyDistancel;
	double critical_long_front_distance =  carInfo.wheel_base/2.0 + carInfo.length/2.0 + params.verticalSafetyDistance;
	double critical_long_back_distance =  carInfo.length/2.0 + params.verticalSafetyDistance - carInfo.wheel_base/2.0;

	PlannerHNS::Mat3 invRotationMat(currState.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(currState.pos.x, currState.pos.y);

	double corner_slide_distance = critical_lateral_distance/2.0;
	double ratio_to_angle = corner_slide_distance/carInfo.max_wheel_angle;
	double slide_distance = vehicleState.steer * ratio_to_angle;

	GPSPoint bottom_left(-critical_lateral_distance ,-critical_long_back_distance,  currState.pos.z, 0);
	GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance,  currState.pos.z, 0);

	GPSPoint top_right_car(critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0,  currState.pos.z, 0);
	GPSPoint top_left_car(-critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0, currState.pos.z, 0);

	GPSPoint top_right(critical_lateral_distance - slide_distance, critical_long_front_distance,  currState.pos.z, 0);
	GPSPoint top_left(-critical_lateral_distance - slide_distance , critical_long_front_distance, currState.pos.z, 0);

	bottom_left = invRotationMat*bottom_left;
	bottom_left = invTranslationMat*bottom_left;

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;

	bottom_right = invRotationMat*bottom_right;
	bottom_right = invTranslationMat*bottom_right;

	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	top_right_car = invRotationMat*top_right_car;
	top_right_car = invTranslationMat*top_right_car;

	top_left_car = invRotationMat*top_left_car;
	top_left_car = invTranslationMat*top_left_car;

	m_SafetyBorder.points.clear();
	m_SafetyBorder.points.push_back(bottom_left) ;
	m_SafetyBorder.points.push_back(bottom_right) ;
	m_SafetyBorder.points.push_back(top_right_car) ;
	m_SafetyBorder.points.push_back(top_right) ;
	m_SafetyBorder.points.push_back(top_left) ;
	m_SafetyBorder.points.push_back(top_left_car) ;

	int iCostIndex = 0;
	if(rollOuts.size() > 0 && rollOuts.at(0).size()>0)
	{
		RelativeInfo car_info;
		PlanningHelpers::GetRelativeInfo(totalPaths, currState, car_info);


		for(unsigned int it=0; it< rollOuts.size(); it++)
		{
			int skip_id = -1;
			for(unsigned int icon = 0; icon < contourPoints.size(); icon++)
			{
				if(skip_id == contourPoints.at(icon).id)
					continue;

				RelativeInfo rel_info;
				PlanningHelpers::GetRelativeInfoLimited(totalPaths, contourPoints.at(icon), rel_info);
				double longitudinalDist = PlanningHelpers::GetExactDistanceOnTrajectory(totalPaths, car_info, rel_info);
				if(rel_info.iFront == 0 && longitudinalDist > 0)
					longitudinalDist = -longitudinalDist;

				double direct_distance = hypot(rel_info.perp_point.pos.y-contourPoints.at(icon).pos.y, rel_info.perp_point.pos.x-contourPoints.at(icon).pos.x);
				if(contourPoints.at(icon).v < params.minSpeed && direct_distance > (m_LateralSkipDistance+contourPoints.at(icon).cost))
				{
					skip_id = contourPoints.at(icon).id;
					continue;
				}

				double lateralDist = fabs(rel_info.perp_distance - trajectoryCosts.at(iCostIndex).distance_from_center);

				if(longitudinalDist < -carInfo.length || longitudinalDist > params.minFollowingDistance || lateralDist > m_LateralSkipDistance)
				{
					continue;
				}

				longitudinalDist = longitudinalDist - critical_long_front_distance;

				if(m_SafetyBorder.PointInsidePolygon(m_SafetyBorder, contourPoints.at(icon).pos) == true)
					trajectoryCosts.at(iCostIndex).bBlocked = true;

				if(lateralDist <= critical_lateral_distance
						&& longitudinalDist >= -carInfo.length/1.5
						&& longitudinalDist < params.minFollowingDistance)
					trajectoryCosts.at(iCostIndex).bBlocked = true;


				if(lateralDist != 0)
					trajectoryCosts.at(iCostIndex).lateral_cost += 1.0/lateralDist;

				if(longitudinalDist != 0)
					trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0/fabs(longitudinalDist);


				if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectoryCosts.at(iCostIndex).closest_obj_distance)
				{
					trajectoryCosts.at(iCostIndex).closest_obj_distance = longitudinalDist;
					trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(icon).v;
				}
			}

			iCostIndex++;
		}
	}
}

void TrajectoryDynamicCosts::NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts)
{
	//Normalize costs
	double totalPriorities = 0;
	double totalChange = 0;
	double totalLateralCosts = 0;
	double totalLongitudinalCosts = 0;
	double transitionCosts = 0;

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		totalPriorities += trajectoryCosts.at(ic).priority_cost;
		transitionCosts += trajectoryCosts.at(ic).transition_cost;
	}

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		totalChange += trajectoryCosts.at(ic).lane_change_cost;
		totalLateralCosts += trajectoryCosts.at(ic).lateral_cost;
		totalLongitudinalCosts += trajectoryCosts.at(ic).longitudinal_cost;
	}

//	cout << "------ Normalizing Step " << endl;
	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		if(totalPriorities != 0)
			trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
		else
			trajectoryCosts.at(ic).priority_cost = 0;

		if(transitionCosts != 0)
			trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / transitionCosts;
		else
			trajectoryCosts.at(ic).transition_cost = 0;

		if(totalChange != 0)
			trajectoryCosts.at(ic).lane_change_cost = trajectoryCosts.at(ic).lane_change_cost / totalChange;
		else
			trajectoryCosts.at(ic).lane_change_cost = 0;

		if(totalLateralCosts != 0)
			trajectoryCosts.at(ic).lateral_cost = trajectoryCosts.at(ic).lateral_cost / totalLateralCosts;
		else
			trajectoryCosts.at(ic).lateral_cost = 0;

		if(totalLongitudinalCosts != 0)
			trajectoryCosts.at(ic).longitudinal_cost = trajectoryCosts.at(ic).longitudinal_cost / totalLongitudinalCosts;
		else
			trajectoryCosts.at(ic).longitudinal_cost = 0;

		trajectoryCosts.at(ic).cost = (m_WeightPriority*trajectoryCosts.at(ic).priority_cost + m_WeightTransition*trajectoryCosts.at(ic).transition_cost + m_WeightLat*trajectoryCosts.at(ic).lateral_cost + m_WeightLong*trajectoryCosts.at(ic).longitudinal_cost)/4.0;

//		cout << "Index: " << ic
//						<< ", Priority: " << trajectoryCosts.at(ic).priority_cost
//						<< ", Transition: " << trajectoryCosts.at(ic).transition_cost
//						<< ", Lat: " << trajectoryCosts.at(ic).lateral_cost
//						<< ", Long: " << trajectoryCosts.at(ic).longitudinal_cost
//						<< ", Change: " << trajectoryCosts.at(ic).lane_change_cost
//						<< ", Avg: " << trajectoryCosts.at(ic).cost
//						<< endl;
	}

//	cout << "------------------------ " << endl;
}

void TrajectoryDynamicCosts::CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params)
{
	for(int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		trajectoryCosts.at(ic).transition_cost = fabs(params.rollOutDensity * (ic - currTrajectoryIndex));
	}
}

}
