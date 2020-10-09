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

#include "op_motion_predictor_core.h"
#include "op_planner/MappingHelpers.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"

namespace MotionPredictorNS
{

MotionPrediction::MotionPrediction()
{
	bMap = false;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bTrackedObjects = false;
	m_bEnableCurbObstacles = false;
	m_DistanceBetweenCurbs = 1.0;
	m_VisualizationTime = 0.25;
	m_bGoNextStep = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_predicted_objects_trajectories = nh.advertise<autoware_msgs::DetectedObjectArray>("/predicted_objects", 1);
	pub_PredictedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("/predicted_trajectories_rviz", 1);
	pub_CurbsRviz					= nh.advertise<visualization_msgs::MarkerArray>("/map_curbs_rviz", 1);
	pub_ParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_particles", 1);
	pub_GeneratedParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("generated_particles", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_behaviors", 1);
	pub_TargetPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("target_points_on_trajs", 1);

//	sub_StepSignal = nh.subscribe("/pred_step_signal", 		1, &MotionPrediction::callbackGetStepForwardSignals, 		this);
	sub_tracked_objects	= nh.subscribe(m_TrackedObjectsTopicName, 	1,	&MotionPrediction::callbackGetTrackedObjects, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 1,	&MotionPrediction::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		sub_robot_odom = nh.subscribe("/carla/ego_vehicle/odometry", 1, &MotionPrediction::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &MotionPrediction::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		sub_can_info = nh.subscribe("/can_info", 1, &MotionPrediction::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = nh.subscribe(velocity_topic, 1, &MotionPrediction::callbackGetVehicleStatus, this);
	}

	UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
	PlannerHNS::ROSHelpers::InitPredMarkers(500, m_PredictedTrajectoriesDummy);
	PlannerHNS::ROSHelpers::InitCurbsMarkers(500, m_CurbsDummy);
	PlannerHNS::ROSHelpers::InitPredParticlesMarkers(1000, m_PredictedParticlesDummy);
	PlannerHNS::ROSHelpers::InitPredParticlesMarkers(2000, m_GeneratedParticlesDummy, true);

	//Mapping Section
	if(m_MapType == PlannerHNS::MAP_AUTOWARE)
	{
		sub_bin_map = nh.subscribe("/lanelet_map_bin", 1, &MotionPrediction::callbackGetLanelet2, this);
		sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &MotionPrediction::callbackGetVMLanes,  this);
		sub_points = nh.subscribe("/vector_map_info/point", 1, &MotionPrediction::callbackGetVMPoints,  this);
		sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &MotionPrediction::callbackGetVMdtLanes,  this);
		sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &MotionPrediction::callbackGetVMIntersections,  this);
		sup_area = nh.subscribe("/vector_map_info/area", 1, &MotionPrediction::callbackGetVMAreas,  this);
		sub_lines = nh.subscribe("/vector_map_info/line", 1, &MotionPrediction::callbackGetVMLines,  this);
		sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &MotionPrediction::callbackGetVMStopLines,  this);
		sub_signals = nh.subscribe("/vector_map_info/signal", 1, &MotionPrediction::callbackGetVMSignal,  this);
		sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &MotionPrediction::callbackGetVMVectors,  this);
		sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &MotionPrediction::callbackGetVMCurbs,  this);
		sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &MotionPrediction::callbackGetVMRoadEdges,  this);
		sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &MotionPrediction::callbackGetVMWayAreas,  this);
		sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &MotionPrediction::callbackGetVMCrossWalks,  this);
		sub_nodes = nh.subscribe("/vector_map_info/node", 1, &MotionPrediction::callbackGetVMNodes,  this);
	}

	std::cout << "OpenPlanner Motion Predictor initialized successfully " << std::endl;
}

MotionPrediction::~MotionPrediction()
{
}

void MotionPrediction::UpdatePlanningParams(ros::NodeHandle& _nh)
{
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

	std::cout << "Rolls Number: " << m_PlanningParams.rollOutNumber << std::endl;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

	_nh.getParam("/op_common_params/front_length", m_CarInfo.front_length);
	_nh.getParam("/op_common_params/back_length", m_CarInfo.back_length);
	_nh.getParam("/op_common_params/height", m_CarInfo.height);
	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;
	else if(iSource == 3)
	{
		m_MapType = PlannerHNS::MAP_LANELET_2;
		std::string str_origin;
		nh.getParam("/op_common_params/lanelet2_origin" , str_origin);
		std::vector<std::string> lat_lon_alt = PlannerHNS::MappingHelpers::SplitString(str_origin, ",");
		if(lat_lon_alt.size() == 3)
		{
			m_Map.origin.pos.lat = atof(lat_lon_alt.at(0).c_str());
			m_Map.origin.pos.lon = atof(lat_lon_alt.at(1).c_str());
			m_Map.origin.pos.alt = atof(lat_lon_alt.at(2).c_str());
		}
	}
	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);

	_nh.getParam("/op_common_params/objects_input_topic" , m_TrackedObjectsTopicName);
	_nh.getParam("/op_common_params/experimentName" , m_ExperimentFolderName);
	if(m_ExperimentFolderName.size() > 0)
	{
		if(m_ExperimentFolderName.at(m_ExperimentFolderName.size()-1) != '/')
			m_ExperimentFolderName.push_back('/');
	}
	UtilityHNS::DataRW::CreateLoggingMainFolder();
	if(m_ExperimentFolderName.size() > 1)
	{
		UtilityHNS::DataRW::CreateExperimentFolder(m_ExperimentFolderName);
	}

	_nh.getParam("/op_motion_predictor/enableGenrateBranches" , m_PredictBeh.m_bGenerateBranches);
	_nh.getParam("/op_motion_predictor/max_distance_to_lane" , m_PredictBeh.m_LaneDetectionDistance);
	_nh.getParam("/op_motion_predictor/min_prediction_distance" , m_PredictBeh.m_MinPredictionDistance);
	_nh.getParam("/op_motion_predictor/min_prediction_time" , m_PredictBeh.m_MinPredictionTime);
	_nh.getParam("/op_motion_predictor/enableCurbObstacles"	, m_bEnableCurbObstacles);
	_nh.getParam("/op_motion_predictor/distanceBetweenCurbs", m_DistanceBetweenCurbs);
	_nh.getParam("/op_motion_predictor/visualizationTime", m_VisualizationTime);
	_nh.getParam("/op_motion_predictor/enableStepByStepSignal", 	m_PredictBeh.m_bStepByStep );
	if(m_PredictBeh.m_bStepByStep)
		m_PredictBeh.m_bDebugOut = true;

	_nh.getParam("/op_motion_predictor/enableParticleFilterPrediction", 	m_PredictBeh.m_bParticleFilter);

	m_PredictBeh.g_PredParams.experiment_name = m_ExperimentFolderName;
	std::cout << "Particles Num Before : " <<  m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM << std::endl;
	_nh.getParam("/op_motion_predictor/pose_weight_factor", 	m_PredictBeh.g_PredParams.POSE_FACTOR);
	_nh.getParam("/op_motion_predictor/dir_weight_factor", 	m_PredictBeh.g_PredParams.DIRECTION_FACTOR);
	_nh.getParam("/op_motion_predictor/vel_weight_factor", 	m_PredictBeh.g_PredParams.VELOCITY_FACTOR);
	_nh.getParam("/op_motion_predictor/acc_weight_factor", 	m_PredictBeh.g_PredParams.ACCELERATE_FACTOR);
	_nh.getParam("/op_motion_predictor/ind_weight_factor", 	m_PredictBeh.g_PredParams.INDICATOR_FACTOR);

	_nh.getParam("/op_motion_predictor/particles_number", 	m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/min_particles_num", 	m_PredictBeh.g_PredParams.MIN_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/keep_percentage", 	m_PredictBeh.g_PredParams.KEEP_PERCENTAGE);
	m_PredictBeh.SetForTrajTracker();

	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
}

void MotionPrediction::callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if(msg->twist.linear.x == 1)
		m_bGoNextStep = true;
	else
		m_bGoNextStep = false;
}

void MotionPrediction::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void MotionPrediction::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	if(msg->twist.twist.linear.x != 0)
		m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
	bVehicleStatus = true;
//	std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

void MotionPrediction::callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);

	autoware_msgs::DetectedObjectArray globalObjects;
	std::string source_data_frame = msg->header.frame_id;
	std::string target_prediction_frame = "/map";

	if(source_data_frame.compare(target_prediction_frame) > 0)
	{
		tf::TransformListener tf_listener;
		tf::StampedTransform local2global;
		PlannerHNS::ROSHelpers::getTransformFromTF(source_data_frame, target_prediction_frame, tf_listener, local2global);
		globalObjects.header = msg->header;
		PlannerHNS::ROSHelpers::transformDetectedObjects(source_data_frame, target_prediction_frame, local2global, *msg, globalObjects);
	}
	else
	{
		globalObjects = *msg;
	}

//	std::cout << std::endl << "New : " << globalObjects.objects.size() << ", Old: " << m_TrackedObjects.size() << std::endl << std::endl;

	m_TrackedObjects.clear();
	bTrackedObjects = true;

	PlannerHNS::DetectedObject obj;
	for(unsigned int i = 0 ; i <globalObjects.objects.size(); i++)
	{
		if(globalObjects.objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(globalObjects.objects.at(i), obj);
			m_TrackedObjects.push_back(obj);
		}
	}

	if(m_PredictBeh.m_bStepByStep && m_bGoNextStep)
	{
		m_bGoNextStep = false;
		m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
	}
	else if(!m_PredictBeh.m_bStepByStep)
	{
		m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
	}

	m_PredictedResultsResults.objects.clear();
	autoware_msgs::DetectedObject pred_obj;
	for(unsigned int i = 0 ; i <m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_PredictBeh.m_ParticleInfo.at(i)->obj, false, pred_obj);
		if(m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track)
			pred_obj.behavior_state = m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track->best_beh_by_p;
		m_PredictedResultsResults.objects.push_back(pred_obj);
	}

	if(m_bEnableCurbObstacles)
	{
		curr_curbs_obstacles.clear();
		GenerateCurbsObstacles(curr_curbs_obstacles);
		PlannerHNS::ROSHelpers::ConvertCurbsMarkers(curr_curbs_obstacles, m_CurbsActual, m_CurbsDummy);
		pub_CurbsRviz.publish(m_CurbsActual);
		//std::cout << "Curbs No: " << curr_curbs_obstacles.size() << endl;
		for(unsigned int i = 0 ; i <curr_curbs_obstacles.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(curr_curbs_obstacles.at(i), false, pred_obj);
			m_PredictedResultsResults.objects.push_back(pred_obj);
		}
	}

	m_PredictedResultsResults.header.stamp = ros::Time().now();
	pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);

	VisualizePrediction();
}

void MotionPrediction::GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles)
{
	if(!bNewCurrentPos) return;
	PlannerHNS::DetectedObject obj;

	for(unsigned int ic = 0; ic < m_Map.curbs.size(); ic++)
	{
		bool bCloseCurb = false;
		for(unsigned int icp=0; icp< m_Map.curbs.at(ic).points.size(); icp++)
		{
			PlannerHNS::WayPoint* pP = &m_Map.curbs.at(ic).points.at(icp);
			double distance = hypot(m_CurrentPos.pos.y - pP->pos.y, m_CurrentPos.pos.x - pP->pos.x);

			if(distance < m_PlanningParams.microPlanDistance)
			{
				bCloseCurb = true;
				break;
			}
		}

		if(bCloseCurb)
		{
			obj.contour.clear();
			for(auto& p: m_Map.curbs.at(ic).points)
			{
				obj.contour.push_back(p.pos);
			}
			PlannerHNS::PlanningHelpers::FixPathDensity(obj.contour, m_DistanceBetweenCurbs);
			obj.bDirection = false;
			obj.bVelocity = false;
			obj.id = -1;
			obj.t  = PlannerHNS::SIDEWALK;
			obj.label = "curb";
			curb_obstacles.push_back(obj);
		}
	}
}

void MotionPrediction::VisualizePrediction()
{
//	m_all_pred_paths.clear();
//	for(unsigned int i=0; i< m_PredictBeh.m_PredictedObjects.size(); i++)
//		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.end());
//
//	PlannerHNS::ROSHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
//	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);
//


	m_all_pred_paths.clear();
	m_particles_points.clear();
	visualization_msgs::MarkerArray behavior_rviz_arr;


	m_TargetPointsOnTrajectories.markers.clear();

	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_ParticleInfo.at(i)->obj.predTrajectories.begin(), m_PredictBeh.m_ParticleInfo.at(i)->obj.predTrajectories.end());

		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.size(); t++)
		{
			PlannerHNS::WayPoint tt_wp = m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->followPoint;
			visualization_msgs::Marker targetPoint = PlannerHNS::ROSHelpers::CreateGenMarker(tt_wp.pos.x,tt_wp.pos.y,tt_wp.pos.z,0,0,0.0,1,0.5,t,"target_trajectory_point", visualization_msgs::Marker::SPHERE);
			m_TargetPointsOnTrajectories.markers.push_back(targetPoint);

			PlannerHNS::WayPoint p_wp;
			for(unsigned int j=0; j < m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.size(); j++)
			{
				PlannerHNS::Particle* pPart = &m_PredictBeh.m_ParticleInfo.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.at(j);
				p_wp = pPart->pose;
				if(pPart->beh == PlannerHNS::BEH_STOPPING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_PARKING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_YIELDING_STATE)
					p_wp.bDir = PlannerHNS::BACKWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_FORWARD_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;

				m_particles_points.push_back(p_wp);
			}
		}

		if(m_PredictBeh.m_ParticleInfo.at(i) != nullptr && m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track != nullptr)
		{
			visualization_msgs::Marker behavior_rviz;
			std::ostringstream ns_beh;
			ns_beh << "pred_beh_state_" << i;
			PlannerHNS::ROSHelpers::VisualizeIntentionState(m_PredictBeh.m_ParticleInfo.at(i)->obj.center, m_PredictBeh.m_ParticleInfo.at(i)->best_behavior_track->best_beh_by_p, behavior_rviz, ns_beh.str(), 3);
			behavior_rviz_arr.markers.push_back(behavior_rviz);
		}
	}
	pub_BehaviorStateRviz.publish(behavior_rviz_arr);


	PlannerHNS::ROSHelpers::ConvertParticles(m_particles_points,m_PredictedParticlesActual, m_PredictedParticlesDummy);
	pub_ParticlesRviz.publish(m_PredictedParticlesActual);

	//std::cout << "Start Tracking of Trajectories : " <<  m_all_pred_paths.size() << endl;
	for(auto& path: m_all_pred_paths)
	{
		PlannerHNS::PlanningHelpers::FixPathDensity(path, 1.0);
	}

	PlannerHNS::ROSHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);

	m_generated_particles_points.clear();
	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo.size(); i++)
	{
		PlannerHNS::WayPoint p_wp;
		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.size(); t++)
		{
			p_wp = m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).pose;
			if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_FORWARD_STATE)
			{
				p_wp.bDir = PlannerHNS::FORWARD_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_YIELDING_STATE)
			{
				p_wp.bDir = PlannerHNS::BACKWARD_DIR;
			}
			m_generated_particles_points.push_back(p_wp);
		}
	}
	PlannerHNS::ROSHelpers::ConvertParticles(m_generated_particles_points,m_GeneratedParticlesActual, m_GeneratedParticlesDummy, true);
	pub_GeneratedParticlesRviz.publish(m_GeneratedParticlesActual);


	pub_TargetPointsRviz.publish(m_TargetPointsOnTrajectories);
}

void MotionPrediction::MainLoop()
{
	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		ros::spinOnce();

		LoadMap();

//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_VisualizationTimer) > m_VisualizationTime)
//		{
//			VisualizePrediction();
//			UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
//		}

		//For the debugging of prediction
//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_SensingTimer) > 5)
//		{
//			ROS_INFO("op_motion_prediction sensing timeout, can't receive tracked object data ! Reset .. Reset");
//			m_PredictedResultsResults.objects.clear();
//			pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);
//		}

		loop_rate.sleep();
	}
}

//Mapping Section

void MotionPrediction::LoadMap()
{
	if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
	{
		bMap = true;
		PlannerHNS::KmlMapLoader kml_loader;
		kml_loader.LoadKML(m_MapPath, m_Map);
		PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	}
	else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
	{
		bMap = true;
		PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
		vec_loader.LoadFromFile(m_MapPath, m_Map);
		PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	}
	else if (m_MapType == PlannerHNS::MAP_LANELET_2 && !bMap)
	{
		bMap = true;
		PlannerHNS::Lanelet2MapLoader map_loader;
		map_loader.LoadMap(m_MapPath, m_Map);
		PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	}
	else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
	{
		if(m_MapRaw.AreMessagesReceived())
		{
			bMap = true;
			PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
			vec_loader.LoadFromData(m_MapRaw, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
	}
}

void MotionPrediction::callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg)
{
	PlannerHNS::Lanelet2MapLoader map_loader;
	map_loader.LoadMap(msg, m_Map);
	PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	bMap = true;
}

void MotionPrediction::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void MotionPrediction::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void MotionPrediction::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void MotionPrediction::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void MotionPrediction::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void MotionPrediction::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void MotionPrediction::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void MotionPrediction::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void MotionPrediction::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void MotionPrediction::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void MotionPrediction::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void MotionPrediction::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void MotionPrediction::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void MotionPrediction::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
