/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_trajectory_evaluator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include <numeric>


namespace TrajectoryEvaluatorNS
{

TrajectoryEvalCore::TrajectoryEvalCore()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;
	bWayGlobalPathToUse = false;
	m_bUseMoveingObjectsPrediction = false;
	bEnableSmoothGlobalPathForCARLA = false;
	m_bKeepCurrentIfPossible = false;
	bNewBehaviorState = false;
	m_AdditionalFollowDistance = 10; // meters

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_CollisionPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("dynamic_collision_points_rviz", 1);
	pub_LocalWeightedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);
	pub_LocalWeightedTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
	pub_TrajectoryCost = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
	pub_SafetyBorderRviz = nh.advertise<visualization_msgs::Marker>("safety_border", 1);

	sub_current_pose = nh.subscribe("/current_pose", 1, &TrajectoryEvalCore::callbackGetCurrentPose, this);

	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		sub_robot_odom = nh.subscribe("/carla/ego_vehicle/odometry", 1, &TrajectoryEvalCore::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &TrajectoryEvalCore::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		sub_can_info = nh.subscribe("/can_info", 1, &TrajectoryEvalCore::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		_nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &TrajectoryEvalCore::callbackGetVehicleStatus, this);
	}

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryEvalCore::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/local_trajectories", 1, &TrajectoryEvalCore::callbackGetLocalPlannerPath, this);
	sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &TrajectoryEvalCore::callbackGetPredictedObjects, this);
	//sub_current_lane_index = nh.subscribe("/op_curr_lane_index", 1, &TrajectoryEvalCore::callbackGetLaneIndex, this);
	//sub_current_trajectory_index = nh.subscribe("/op_curr_trajectory_index", 1, &TrajectoryEvalCore::callbackGetTrajectoryIndex, this);
	sub_behavior_state = _nh.subscribe("/op_current_behavior",	1, &TrajectoryEvalCore::callbackGetBehaviorState, 	this);

	m_TrajectoryCostsCalculator.SetEvalParams(m_EvaluationParams);
	PlannerHNS::ROSHelpers::InitCollisionPointsMarkers(500, m_CollisionsDummy);
}

TrajectoryEvalCore::~TrajectoryEvalCore()
{
}

void TrajectoryEvalCore::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_trajectory_evaluator/enablePrediction", m_bUseMoveingObjectsPrediction);
	_nh.getParam("/op_trajectory_evaluator/keepCurrentTrajectory", m_bKeepCurrentIfPossible);


	_nh.getParam("/op_trajectory_evaluator/weight_priority", m_EvaluationParams.priority_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_transition", m_EvaluationParams.transition_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_longitudinal", m_EvaluationParams.longitudinal_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_lateral", m_EvaluationParams.lateral_weight_);
	_nh.getParam("/op_trajectory_evaluator/weight_lane_change", m_EvaluationParams.lane_change_weight_);
	_nh.getParam("/op_trajectory_evaluator/collision_time", m_EvaluationParams.collision_time_);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);
	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);
	if(!m_PlanningParams.enableLaneChange)
	{
		m_EvaluationParams.lane_change_weight_ = 0;
		BalanceFactorsToOne(m_EvaluationParams.priority_weight_, m_EvaluationParams.transition_weight_,
				m_EvaluationParams.longitudinal_weight_, m_EvaluationParams.lateral_weight_, m_EvaluationParams.lane_change_weight_);

	}

	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;
}

void TrajectoryEvalCore::BalanceFactorsToOne(double& priority, double& transition, double& longitudinal, double& lateral, double& change)
	{
	std::vector<double> factors_list = {priority, transition, longitudinal, lateral, change};
	int nNonZero = 0;
	for(unsigned int i=0;i < factors_list.size(); i++)
	{
		if(factors_list.at(i) > 0.0)
			nNonZero++;
	}
	double all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

	while(all_factors > 1.01 || all_factors < 0.99)
	{
		double every_one_share = (1.0 - all_factors)/(double)nNonZero;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
			{
				factors_list.at(i) += every_one_share;
			}

			if(factors_list.at(i) < 0.0)
				factors_list.at(i) = 0.0;
		}

		nNonZero = 0;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
				nNonZero++;
		}
		all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

		if(all_factors == 0)
			break;
	}
	priority = factors_list.at(0);
	transition = factors_list.at(1);
	longitudinal = factors_list.at(2);
	lateral = factors_list.at(3);
	change = factors_list.at(4);

}

void TrajectoryEvalCore::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void TrajectoryEvalCore::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.twist.linear.x) > 0.1)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEvalCore::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
	bVehicleStatus = true;
	//std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

void TrajectoryEvalCore::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_GlobalPaths.clear();
		prev_lane_idx = -1; // woocheol
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);
			m_GlobalPaths.push_back(m_temp_path);
		}

		bool bOldGlobalPath = true;
		if(m_GlobalPathsToUse.size() == m_GlobalPaths.size())
		{
			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_GlobalPaths.at(i), m_GlobalPathsToUse.at(i));
			}
		}
		else
		{
			bOldGlobalPath = false;
		}

		if(!bOldGlobalPath)
		{
			if(bEnableSmoothGlobalPathForCARLA)
			{
				for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
				}
			}
			else
			{
				for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
				}
			}

			bWayGlobalPath = true;

			std::cout << "Received New Global Paths Evaluator ! " << m_GlobalPaths.size() << std::endl;
		}
	}
}

void TrajectoryEvalCore::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_GeneratedRollOuts.clear();
		std::vector<int> globalPathsId_roll_outs;

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
			m_GeneratedRollOuts.push_back(path);

			int roll_out_gid = -1;
			if(path.size() > 0)
			{
				roll_out_gid = path.at(0).gid;
			}

			if(std::find(globalPathsId_roll_outs.begin(), globalPathsId_roll_outs.end(), roll_out_gid) == globalPathsId_roll_outs.end())
			{
				globalPathsId_roll_outs.push_back(roll_out_gid);
			}
		}

		if(globalPathsId_roll_outs.size() != m_GlobalPathsToUse.size())
		{
			std::cout << "Warning From Trajectory Evaluator, paths size mismatch, GlobalPaths: " << m_GlobalPathsToUse.size() << ", LocalPaths: " << globalPathsId_roll_outs.size() << std::endl;
			bWayGlobalPath = true;
 		}

		if(bWayGlobalPath)
		{
			m_GlobalPathsToUse.clear();
			m_prev_index.clear();
			for(unsigned int i=0; i < globalPathsId_roll_outs.size(); i++)
			{
				for(unsigned int j=0; j < m_GlobalPaths.size(); j++)
				{
					if(m_GlobalPaths.at(j).size() > 0)
					{
						std::cout << "Before Synchronization At Trajectory Evaluator: GlobalID: " <<  m_GlobalPaths.at(j).at(0).gid << ", LocalID: " << globalPathsId_roll_outs.at(i) << std::endl;
						if(m_GlobalPaths.at(j).at(0).gid == globalPathsId_roll_outs.at(i))
						{
							bWayGlobalPath = false;
							m_GlobalPathsToUse.push_back(m_GlobalPaths.at(j));
							m_prev_index.push_back(0);
							std::cout << "Synchronization At Trajectory Evaluator: GlobalID: " <<  m_GlobalPaths.at(j).at(0).gid << ", LocalID: " << globalPathsId_roll_outs.at(i) << std::endl;
							break;
						}
					}
				}
			}
		}

		bRollOuts = true;
	}
}

void TrajectoryEvalCore::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	bPredictedObjects = true;

	PlannerHNS::DetectedObject obj;
	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
			m_PredictedObjects.push_back(obj);
		}
//		else
//		{
//			std::cout << " Ego Car avoid detecting itself in trajectory evaluator node! ID: " << msg->objects.at(i).id << std::endl;
//		}
	}
}

//void TrajectoryEvalCore::callbackGetTrajectoryIndex(const std_msgs::Int32ConstPtr& msg)
//{
//	m_CurrentBehavior.iTrajectory = msg->data;
//}
//
//void TrajectoryEvalCore::callbackGetLaneIndex(const std_msgs::Int32ConstPtr& msg)
//{
//	m_CurrentBehavior.iLane = msg->data;
//}

void TrajectoryEvalCore::callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg )
{
	m_CurrentBehavior = PlannerHNS::ROSHelpers::ConvertAutowareWaypointToBehaviorState(*msg);
	//std::cout << "Receive Behavior State : " << m_CurrentBehavior.state << ", Target Speed: " << m_CurrentBehavior.maxVelocity << ", StopD: " << m_CurrentBehavior.stopDistance << ", FollowD: " << m_CurrentBehavior.followDistance << std::endl;
	bNewBehaviorState = true;
}

void TrajectoryEvalCore::CollectRollOutsByGlobalPath()
{
	m_LanesRollOuts.clear();
	std::vector< std::vector<PlannerHNS::WayPoint> > local_category;
	for(auto& g_path: m_GlobalPathsToUse)
	{
		if(g_path.size() > 0)
		{
			local_category.clear();
			for(auto& l_traj: m_GeneratedRollOuts)
			{
				if(l_traj.size() > 0 && l_traj.at(0).gid == g_path.at(0).gid)
				{
					local_category.push_back(l_traj);
					//std::cout << "Costs Between Global And Generated Local: Global Cost: " << g_path.at(0).laneChangeCost << ", Local Cost: " << l_traj.at(0).laneChangeCost << std::endl;
				}
			}
			m_LanesRollOuts.push_back(local_category);
		}
	}
}

bool TrajectoryEvalCore::FindBestLane(std::vector<PlannerHNS::TrajectoryCost> tcs, PlannerHNS::TrajectoryCost& best_l)
{
	if(tcs.size() == 0) return false;

	if(tcs.size() == 1)
	{
		best_l = tcs.at(0);
		return true;
	}

	std::sort(tcs.begin(), tcs.end(), PlannerHNS::TrajectoryEvaluator::sortCosts);

	for(unsigned int i=0 ; i < tcs.size(); i++)
	{
		if(!tcs.at(i).bBlocked)
		{
			best_l = tcs.at(i);
			break;
		}
	}

	if(best_l.bBlocked) // if the best lane is blocked , keep the previous lane as the best lane
	{
		if(m_CurrentBehavior.iLane >= 0 && m_CurrentBehavior.iLane < tcs.size())
		{
			best_l = tcs.at(m_CurrentBehavior.iLane);
		}
		else
		{
			best_l = tcs.at(0);
		}
	}

	return true;
}

void TrajectoryEvalCore::setFollowDistance()
{
	double min_limit_distance		= 30.0;
	double m_kph					= m_CurrentPos.v * 3.6;
	double ichthus_latency_margin 	= 0.25 * m_CurrentPos.v*m_CurrentPos.v + 0.1 * m_CurrentPos.v; //(10m/s : 26m, 15m/s : 55m)
	double m_brake_distance 		= 0.005 * m_kph*m_kph + 0.2 * m_kph; // origin car braking distance (10m/s => 13.68m)
	double m_free_running_distance 	= m_CurrentPos.v; 
	double m_newFollowDistance 		= m_brake_distance + m_free_running_distance + ichthus_latency_margin;
	m_newFollowDistance = std::max(min_limit_distance, m_newFollowDistance);
	// ROS_INFO("m_newFollowDistance : %f", m_newFollowDistance);
	m_PlanningParams.minFollowingDistance = m_newFollowDistance;
	// nh.setParam("/op_common_params/minFollowingDistance", m_newFollowDistance);
} // woocheol

bool TrajectoryEvalCore::isSameLaneSession(PlannerHNS::TrajectoryCost& prev_lane_costs, PlannerHNS::TrajectoryCost& best_lane_costs)
{
	if(prev_lane_costs.index == -1) return true;
	int m_rollouts = m_PlanningParams.rollOutNumber + 1;
	int prev_lane_session = prev_lane_costs.index / m_rollouts; // if m_rollouts : 5 and lane idx : 0~4, lane session : 0
	int best_lane_session = best_lane_costs.index / m_rollouts;
	return (prev_lane_session == best_lane_session);
}// woocheol

void TrajectoryEvalCore::isChanged(const int& prev_lane_idx, const int& curr_lane_idx)
{
	ROS_INFO("prev_lane_idx : %d", prev_lane_idx);
	ROS_INFO("curr_lane_idx : %d", curr_lane_idx);
	int m_rollouts = m_PlanningParams.rollOutNumber + 1;
	int prev_lane_session = prev_lane_idx / m_rollouts;
	int curr_lane_session = curr_lane_idx / m_rollouts;
	is_changed = (prev_lane_session != curr_lane_session);

	if(prev_lane_idx == -1){
		is_changed = false;
	}
	ROS_INFO("prev_lane_session : %d", prev_lane_session);
	ROS_INFO("curr_lane_session : %d", curr_lane_session);
	ROS_INFO("is_changed : %d", is_changed);
}// woocheol

void TrajectoryEvalCore::MainLoop()
{
	ros::Rate loop_rate(50);

	PlannerHNS::WayPoint prevState, state_change;
	while (ros::ok())
	{
		ros::spinOnce();

		setFollowDistance(); // woocheol
		if(bNewCurrentPos)
		{
			m_GlobalPathSections.clear();

			if(m_prev_index.size() != m_GlobalPathsToUse.size())
			{
				m_prev_index.clear();
				for(unsigned int i=0; i < m_GlobalPathsToUse.size(); i++)
				{
					m_prev_index.push_back(0);
				}
			}

			for(unsigned int i = 0; i < m_GlobalPathsToUse.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPathsToUse.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,
						m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed, m_prev_index.at(i));

				if(m_prev_index.at(i) > 0 ) m_prev_index.at(i) = m_prev_index.at(i) -1;

				m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
			}

			if(m_GlobalPathSections.size()>0)
			{
				autoware_msgs::LaneArray local_lanes;
				std::vector<PlannerHNS::TrajectoryCost> tcs;
				visualization_msgs::Marker safety_box;
				std::vector<PlannerHNS::WayPoint> collision_points;
				visualization_msgs::MarkerArray all_rollOuts;
//				std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > collected_local_roll_outs;
//				std::vector<std::vector<PlannerHNS::TrajectoryCost> > collected_trajectory_costs;

				CollectRollOutsByGlobalPath();
				if(!m_PlanningParams.enableLaneChange)
				{
					PlannerHNS::PlanningParams planningParams = m_PlanningParams;
					if(m_CurrentBehavior.state == PlannerHNS::FOLLOW_STATE)
					{
						planningParams.minFollowingDistance += m_AdditionalFollowDistance;
					}

					PlannerHNS::TrajectoryCost tc = m_TrajectoryCostsCalculator.doOneStep(m_LanesRollOuts.at(0), m_GlobalPathSections.at(0), m_CurrentPos,
							planningParams, m_CarInfo,m_VehicleStatus, m_PredictedObjects, !m_bUseMoveingObjectsPrediction, m_CurrentBehavior.iTrajectory, m_bKeepCurrentIfPossible);
					tcs.push_back(tc);

					for(unsigned int i=0; i < m_TrajectoryCostsCalculator.local_roll_outs_.size(); i++)
					{
							autoware_msgs::Lane lane;
							PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_TrajectoryCostsCalculator.local_roll_outs_.at(i), lane);
							lane.closest_object_distance = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_distance;
							lane.closest_object_velocity = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_velocity;
							lane.cost = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).cost;
							lane.is_blocked = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked;
							lane.lane_index = local_lanes.lanes.size();
							lane.lane_id = 0;
							local_lanes.lanes.push_back(lane);
					}

//					collected_local_roll_outs.push_back(m_TrajectoryCostsCalculator.local_roll_outs_);
//					collected_trajectory_costs.push_back(m_TrajectoryCostsCalculator.trajectory_costs_);
					PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_TrajectoryCostsCalculator.local_roll_outs_, m_TrajectoryCostsCalculator.trajectory_costs_, tc.index, all_rollOuts);
					collision_points.insert(collision_points.end(), m_TrajectoryCostsCalculator.collision_points_.begin(), m_TrajectoryCostsCalculator.collision_points_.end());
					PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.safety_border_.points, safety_box);

				}
				else
				{
					PlannerHNS::PlanningParams planningParams = m_PlanningParams;
					if(m_CurrentBehavior.state == PlannerHNS::FOLLOW_STATE)
					{
						planningParams.minFollowingDistance += m_AdditionalFollowDistance;
					}

					//std::cout << "Start New Evaluations --------------------------- vvvvvvvvvvvvvvvv " <<  std::endl;
					for(unsigned int ig = 0; ig < m_GlobalPathSections.size(); ig++)
					{
//						std::cout << "Best Lane From Behavior Selector: " << m_CurrentBehavior.iLane << ", Trajectory: " << m_CurrentBehavior.iTrajectory << ", Curr Lane: " << ig << std::endl;

						PlannerHNS::TrajectoryCost temp_tc = m_TrajectoryCostsCalculator.doOneStep(m_LanesRollOuts.at(ig), m_GlobalPathSections.at(ig), m_CurrentPos,
								planningParams, m_CarInfo, m_VehicleStatus, m_PredictedObjects, !m_bUseMoveingObjectsPrediction, m_CurrentBehavior.iTrajectory, m_bKeepCurrentIfPossible);


						// woocheol
						if(m_GlobalPathSections.size() > 2 && ig > 1) temp_tc.bBlocked = true;
						// if(int(prev_lane_idx / m_PlanningParams.rollOutNumber + 1)==0 && ig == 0) temp_tc.bBlocked = true;
			


						if(m_GlobalPathSections.at(ig).size() > 0)
						{
							temp_tc.lane_change_cost = m_GlobalPathSections.at(ig).at(0).laneChangeCost;
						}

						temp_tc.lane_index = ig;
						tcs.push_back(temp_tc);

						for(unsigned int i=0; i < m_TrajectoryCostsCalculator.local_roll_outs_.size(); i++)
						{
								autoware_msgs::Lane lane;
								PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_TrajectoryCostsCalculator.local_roll_outs_.at(i), lane);
								lane.closest_object_distance = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_distance;
								lane.closest_object_velocity = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).closest_obj_velocity;
								lane.cost = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).cost;
								// woocheol
								if(m_GlobalPathSections.size() > 2 && ig > 1) m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked = true;
								// if(int(prev_lane_idx / m_PlanningParams.rollOutNumber + 1)==0 && ig == 0) m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked = true;
								lane.is_blocked = m_TrajectoryCostsCalculator.trajectory_costs_.at(i).bBlocked;
								lane.lane_index = local_lanes.lanes.size();
								lane.lane_id = ig;
								local_lanes.lanes.push_back(lane);
						}

						PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_TrajectoryCostsCalculator.local_roll_outs_, m_TrajectoryCostsCalculator.trajectory_costs_, temp_tc.index, all_rollOuts);
						collision_points.insert(collision_points.end(), m_TrajectoryCostsCalculator.collision_points_.begin(), m_TrajectoryCostsCalculator.collision_points_.end());
						PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.safety_border_.points, safety_box);
					}

					PlannerHNS::EvaluationParams eval_params_for_lane_change = m_EvaluationParams;
					eval_params_for_lane_change.lane_change_weight_ = 0.6;
					BalanceFactorsToOne(eval_params_for_lane_change.priority_weight_, eval_params_for_lane_change.transition_weight_,
								eval_params_for_lane_change.longitudinal_weight_, eval_params_for_lane_change.lateral_weight_, eval_params_for_lane_change.lane_change_weight_);
					m_TrajectoryCostsCalculator.normalizeCosts(eval_params_for_lane_change, tcs);
				}

				PlannerHNS::TrajectoryCost best_lane_costs;
				if(FindBestLane(tcs, best_lane_costs))
				{
					autoware_msgs::Lane l;

					/* // simple logic of lane change deceleration./
					if(isSameLaneSession(prev_best_lane_costs, best_lane_costs))
					{
						l.closest_object_distance = best_lane_costs.closest_obj_distance;
						l.closest_object_velocity = best_lane_costs.closest_obj_velocity;
						l.cost					  = best_lane_costs.cost;
						l.is_blocked			  = best_lane_costs.bBlocked;
						l.lane_index			  = best_lane_costs.index;
						l.lane_id				  = best_lane_costs.lane_index;
						prev_best_lane_costs = best_lane_costs;
					}
					else
					{
						l.closest_object_distance = prev_best_lane_costs.closest_obj_distance;
						l.closest_object_velocity = prev_best_lane_costs.closest_obj_velocity;
						l.cost					  = prev_best_lane_costs.cost;
						l.is_blocked 			  = prev_best_lane_costs.bBlocked;
						l.lane_index			  = prev_best_lane_costs.index;
						l.lane_id				  = prev_best_lane_costs.lane_index;
						(input deceleration current velocity code.)
						continue_cnt++;
					}

					if(continue_cnt > 50) // Mainloop hz is 50hz. so continue_cnt 50 per 1 sec. 
					{
						l.closest_object_distance = best_lane_costs.closest_obj_distance;
						l.closest_object_velocity = best_lane_costs.closest_obj_velocity;
						l.cost					  = best_lane_costs.cost;
						l.is_blocked			  = best_lane_costs.bBlocked;
						l.lane_index			  = best_lane_costs.index;
						l.lane_id				  = best_lane_costs.lane_index;
						continue_cnt = 0;
					}
					pub_TrajectoryCost.publish(l);
					*/
					
					/* lane sesstion : lane_id */
					l.closest_object_distance = best_lane_costs.closest_obj_distance;
					l.closest_object_velocity = best_lane_costs.closest_obj_velocity;
					l.cost = best_lane_costs.cost;
					l.is_blocked = best_lane_costs.bBlocked;
					l.lane_index = best_lane_costs.index;
					l.lane_id = best_lane_costs.lane_index;
					
					// isChanged(prev_lane_idx, l.lane_index); // woocheol
					// prev_lane_idx = l.lane_index; // woocheol
					
					pub_TrajectoryCost.publish(l);
				}
				else
				{
					std::cout << "Warning from Trajectory Evaluator, Can't find suitable lane for driving !! " << std::endl;
				}

//				std::cout << "Costs For Lanes:  ---------------------- " << std::endl;
//				std::cout << "Best Lane From Behavior Selector: " << m_CurrentBehavior.iLane << ", Trajectory: " << m_CurrentBehavior.iTrajectory << ", Globals: " << m_GlobalPathSections.size() << ", Locals: " << m_LanesRollOuts.size() <<   std::endl;
//
////				if(m_CurrentBehavior.iLane >= 0 && m_CurrentBehavior.iLane < m_GlobalPathSections.size())
////				{
////					std::cout << "Lane Change Cost: for Current Lane: " << m_LanesRollOuts.at(m_CurrentBehavior.iLane).at(m_CurrentBehavior.iTrajectory).at(0).laneChangeCost << std::endl;
////				}
//				for(unsigned int i=0; i < tcs.size(); i++)
//				{
//					std::cout << "i: " << i << ", Index: " << tcs.at(i).index << ", GlobalIndex: " << tcs.at(i).lane_index << ", Cost: " << tcs.at(i).cost << ", Blocked: " << tcs.at(i).bBlocked << std::endl;
//				}
//				std::cout << "Best Lane From Cost Calculator : " <<std::endl;
//				std::cout << "i: " << -1 << ", Index: " << best_lane_costs.index << ", GlobalIndex: " << best_lane_costs.lane_index << ", Cost: " << best_lane_costs.cost << ", Blocked: " << best_lane_costs.bBlocked << std::endl;
//				std::cout << "---------------------- " << std::endl;

				pub_LocalWeightedTrajectories.publish(local_lanes);

				PlannerHNS::ROSHelpers::ConvertCollisionPointsMarkers(collision_points, m_CollisionsActual, m_CollisionsDummy);
				pub_CollisionPointsRviz.publish(m_CollisionsActual);

				pub_SafetyBorderRviz.publish(safety_box);
//				PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(collected_local_roll_outs.at(best_lane_costs.lane_index), collected_trajectory_costs.at(best_lane_costs.lane_index), best_lane_costs.index, all_rollOuts);
				pub_LocalWeightedTrajectoriesRviz.publish(all_rollOuts);
			}
		}
		else
		{
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryEvalCore::callbackGetGlobalPlannerPath, 	this);
		}

		loop_rate.sleep();
	}
}

}
