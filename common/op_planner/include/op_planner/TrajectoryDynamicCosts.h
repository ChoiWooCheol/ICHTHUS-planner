
/// \file TrajectoryDynamicCosts.h
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+
/// \author Hatem Darweesh
/// \date Jan 14, 2018
/// Use only in Simu Decision Maker to calculate obstacle avoidance for the simulated cars


#ifndef TRAJECTORYDYNAMICCOSTS_H_
#define TRAJECTORYDYNAMICCOSTS_H_

#include "RoadNetwork.h"
#include "PlannerCommonDef.h"
#include "PlanningHelpers.h"

using namespace std;

namespace PlannerHNS
{

class TrajectoryDynamicCosts
{
public:
	TrajectoryDynamicCosts();
	virtual ~TrajectoryDynamicCosts();

	TrajectoryCost DoOneStepStatic(const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths,
			const WayPoint& currState, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState,
			const std::vector<PlannerHNS::DetectedObject>& obj_list, const int& iCurrentIndex = -1);

public:
	int m_PrevCostIndex;
	int m_PrevIndex;
	vector<TrajectoryCost> m_TrajectoryCosts;
	PlanningParams m_Params;
	PolygonShape m_SafetyBorder;
	vector<WayPoint> m_AllContourPoints;
	double m_WeightPriority;
	double m_WeightTransition;
	double m_WeightLong;
	double m_WeightLat;
	double m_LateralSkipDistance;

private:
	void NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts);
	void CalculateLateralAndLongitudinalCostsStatic(vector<TrajectoryCost>& trajectoryCosts, const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths, const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState);
	void CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params);
};

}

#endif /* TRAJECTORYDYNAMICCOSTS_H_ */
