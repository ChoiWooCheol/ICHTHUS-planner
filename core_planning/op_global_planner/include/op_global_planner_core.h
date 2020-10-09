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

#ifndef OP_GLOBAL_PLANNER
#define OP_GLOBAL_PLANNER

#include <ros/ros.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/State.h>
#include <autoware_msgs/VehicleStatus.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_msgs/Int8.h>
#include "autoware_can_msgs/CANInfo.h"
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include "op_planner/hmi/HMIMSG.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerH.h"
#include "op_utility/DataRW.h"

#include <geometry_msgs/PoseArray.h> // woocheol
#include <geometry_msgs/Pose.h> // woocheol

namespace GlobalPlanningNS
{

#define MAX_GLOBAL_PLAN_SEARCH_DISTANCE 100000 //meters
#define MIN_EXTRA_PLAN_DISTANCE 120 //meters
//#define REPLANNING_DISTANCE 2.5
//#define REPLANNING_TIME 5

class GlobalPlanningParams
{
public:
	PlannerHNS::MAP_SOURCE_TYPE	mapSource; //which map source user wants to select, using autoware loaded map, load vector mapt from file, use kml map file, use lanelet2 map file.
	std::string mapPath; //path to map file or folder, depending on the mapSource parameter
	std::string exprimentName; //folder name that will contains generated global path logs, when new global path is generated a .csv file will be written.
	std::string destinationsFile; //file path of the list of destinations for the global path to achieve.
	bool bEnableSmoothing; //additional smoothing step to the generated global path, of the waypoints are far apart, this could lead to corners cutting.
	bool bEnableLaneChange; //Enable general multiple global paths to enable lane change
	bool bEnableHMI; // Enable communicating with third party HMI client, to receive outside commands such as go to specific destination, slow down, etc ..
	bool bEnableRvizInput; //Using this will ignore reading the destinations file. GP will wait for user input to Rviz, user can select one start position and multiple destinations positions.
	bool bEnableReplanning; //Enable going into an infinite loop of global planning, when the final destination is reached, the GP will plan a path from it to the first destination.
	double pathDensity; //Used only when enableSmoothing is enabled, the maximum distance between each two waypoints in the generated path
	int waitingTime; // waiting time at each destination in seconds.
	double endOfPathDistance; // when the vehicle is close to the end of global path with this distance , the waiting state will triggered
	double slowDownSpeed; // when HMI send slow down command, this speed will be assigned to the new trajectory, in km/hour

	GlobalPlanningParams()
	{
		waitingTime = 4;
		bEnableReplanning = false;
		bEnableHMI = false;
		bEnableSmoothing = false;
		bEnableLaneChange = false;
		bEnableRvizInput = true;
		pathDensity = 0.5;
		mapSource = PlannerHNS::MAP_KML_FILE;
		endOfPathDistance = 0.5;
		slowDownSpeed = 15;
	}
};


class GlobalPlanner
{

public:
	int m_iCurrentGoalIndex;
	int m_HMIDestinationID;
protected:
	std::vector<PlannerHNS::WayPoint> m_branch_points; // woocheol
	ros::Publisher pub_BranchPose; // woocheol
	ros::Publisher pub_BranchPoseRviz; // woocheol
	void publishBranchPointArray(); // woocheol

	GlobalPlanningParams m_params;
	PlannerHNS::WayPoint m_CurrentPose;
	std::vector<PlannerHNS::WayPoint> m_GoalsPos;
	PlannerHNS::WayPoint m_StartPose;
	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::VehicleState m_VehicleState;
	int m_GlobalPathID;
	std::vector<int> m_prev_index;
	int m_iMessageID;
	bool m_bFirstStartHMI;
	bool m_bWaitingState;
	bool m_bSlowDownState;
	bool m_bStoppingState;
	bool m_bReStartState;
	bool m_bDestinationError;
	timespec m_WaitingTimer;
	timespec m_ReplanningTimer;
	bool m_bReplanSignal;

	PlannerHNS::WayPoint m_PreviousPlanningPose;

	ros::NodeHandle nh;

	ros::Publisher pub_MapRviz;
	ros::Publisher pub_Paths;
	ros::Publisher pub_PathsRviz;
	ros::Publisher pub_GoalsListRviz;
	ros::Publisher pub_hmi_mission;

	ros::Subscriber sub_replan_signal;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_vehicle_status;
	ros::Subscriber sub_start_pose;
	ros::Subscriber sub_goal_pose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_road_status_occupancy;
	ros::Subscriber sub_hmi_mission;
	ros::Subscriber sub_map_file_name;

public:
	GlobalPlanner();
  ~GlobalPlanner();
  void MainLoop();

private:
  std::vector<UtilityHNS::DestinationsDataFileReader::DestinationData> m_destinations;

  // Callback function for subscriber.
  void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
  void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
  void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);
  void callbackGetReplanSignal(const std_msgs::BoolConstPtr& msg);
  /**
   * @brief Communication between Global Planner and HMI bridge
   * @param msg
   */
  void callbackGetHMIState(const autoware_msgs::StateConstPtr& msg);

  protected:
  	PlannerHNS::RoadNetwork m_Map;
  	bool m_bMap;
  	PlannerHNS::PlannerH m_PlannerH;
  	std::vector<std::vector<PlannerHNS::WayPoint> > m_GeneratedTotalPaths;

  	bool GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths);
  	void VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths);
  	void VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected);
  	void SaveSimulationData();
  	int LoadSimulationData();
  	void LoadDestinations(const std::string& fileName);
  	int CheckForEndOfPaths(const std::vector<std::vector<PlannerHNS::WayPoint> >& paths, const PlannerHNS::WayPoint& currPose, const double& end_range_distance);
  	void FindIncommingBranches(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, const PlannerHNS::WayPoint& currPose,const double& min_distance,const double& max_distance,
  				std::vector<PlannerHNS::WayPoint*>& branches);
  	PlannerHNS::ACTION_TYPE FromMsgAction(const PlannerHNS::MSG_ACTION& msg_action);
  	PlannerHNS::MSG_ACTION ToMsgAction(const PlannerHNS::ACTION_TYPE& action);
  	void SendAvailableOptionsHMI();
  	bool UpdateGoalIndex();
  	bool UpdateGoalWithHMI();


  	//Mapping Section
  	UtilityHNS::MapRaw m_MapRaw;
  	ros::Subscriber sub_bin_map;
	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;


	void callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg);
	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
	void kmlMapFileNameCallback(const std_msgs::String& file_name);
	void LoadKmlMap();
	void LoadMap();

	/**
	 * Animate Global path generation
	 */
	PlannerHNS::WayPoint* m_pCurrGoal;
  	ros::Publisher pub_GlobalPlanAnimationRviz;
  	std::vector<PlannerHNS::WayPoint*> m_PlanningVisualizeTree;
  	std::vector<PlannerHNS::WayPoint*> m_CurrentLevel;
  	visualization_msgs::MarkerArray m_AccumPlanLevels;
  	unsigned int m_iCurrLevel;
  	unsigned int m_nLevelSize;
  	double m_CurrMaxCost;
  	int m_bSwitch;
  	bool m_bEnableAnimation;
  	void AnimatedVisualizationForGlobalPath(double time_interval = 0.5);
  	timespec m_animation_timer;

};

}

#endif
