/*
 * VectorMapLoader.cpp
 *
 *  Created on: Mar 16, 2020
 *      Author: hatem
 */

#include <op_planner/VectorMapLoader.h>
#include <op_planner/PlanningHelpers.h>

namespace PlannerHNS {
#define RIGHT_INITIAL_TURNS_COST 0
#define LEFT_INITIAL_TURNS_COST 0
#define DEFAULT_REF_VELOCITY 60 //km/h

VectorMapLoader::VectorMapLoader(int map_version, bool enable_lane_change, bool load_curbs, bool load_lines, bool load_wayareas) :
		_map_version(map_version), _find_parallel_lanes(enable_lane_change), _load_curbs(load_curbs), _load_lines(load_lines), _load_wayareas(load_wayareas) {
}

VectorMapLoader::~VectorMapLoader() {
}

void VectorMapLoader::SetAdditionalOrigin(const PlannerHNS::WayPoint& origin)
{
	_origin = origin;
}

void VectorMapLoader::LoadFromFile(const std::string& vectorMapFolder, PlannerHNS::RoadNetwork& map)
{
	map.Clear();
	UtilityHNS::MapRaw map_raw;
	map_raw.LoadFromFolder(vectorMapFolder);
	LoadFromData(map_raw, map);
}

void VectorMapLoader::LoadFromData(UtilityHNS::MapRaw& mapRaw, PlannerHNS::RoadNetwork& map)
{
	map.Clear();
	if(_map_version == 0)
	{
		ConstructRoadNetworkFromROSMessageVer0(mapRaw, _origin, map);
	}
	else
	{
		ConstructRoadNetworkFromROSMessage(mapRaw, _origin, map);
	}

	WayPoint origin = MappingHelpers::GetFirstWaypoint(map);
	std::cout << "First Point: " <<  origin.pos.ToString();
}

void VectorMapLoader::ConstructRoadNetworkFromROSMessage(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map)
{
	std::vector<Lane> roadLanes;
	for(unsigned int i=0; i< mapRaw.pLanes->m_data_list.size(); i++)
	{
		if(mapRaw.pLanes->m_data_list.at(i).LnID > RoadNetwork::g_max_lane_id)
			RoadNetwork::g_max_lane_id = mapRaw.pLanes->m_data_list.at(i).LnID;
	}

	std::cout << std::endl << " >> Extracting Lanes ... " << std::endl;
	CreateLanes(mapRaw.pLanes, mapRaw.pPoints, mapRaw.pNodes , roadLanes);

	std::cout << " >> Fix Waypoints errors ... " << std::endl;
	MappingHelpers::FixTwoPointsLanes(roadLanes);
	MappingHelpers::FixRedundantPointsLanes(roadLanes);
	ConnectLanes(mapRaw.pLanes, roadLanes);

	std::cout << " >> Create Missing lane connections ... " << std::endl;
	MappingHelpers::FixUnconnectedLanes(roadLanes);
	////FixTwoPointsLanes(roadLanes);

	//map has one road segment
	map.roadSegments.push_back(RoadSegment());
	map.roadSegments.at(0).Lanes = roadLanes;

	//Fix angle for lanes
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);
		}
	}

	//Link Lanes and lane's waypoints by pointers
	std::cout << " >> Link lanes and waypoints with pointers ... " << std::endl;
	MappingHelpers::LinkLanesPointers(map);

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
			    if(pL->points.at(j).actionCost.size() > 0)
			  {
				  if(pL->points.at(j).actionCost.at(0).first == LEFT_TURN_ACTION)
					{
					  MappingHelpers::AssignActionCostToLane(pL, LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST);
					  break;
					}
				  else if(pL->points.at(j).actionCost.at(0).first == RIGHT_TURN_ACTION)
					{
					  MappingHelpers::AssignActionCostToLane(pL, RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST);
					break;

					}
				}
			}
		}
	}

	if(_find_parallel_lanes)
	{
		std::cout << " >> Extract Lane Change Information... " << std::endl;
		MappingHelpers::FindAdjacentLanesV2(map);
	}

	//Extract Signals and StopLines
	std::cout << " >> Extract Signal data ... " << std::endl;
	ExtractSignalDataV2(mapRaw.pSignals->m_data_list, mapRaw.pVectors->m_data_list, mapRaw.pPoints, origin.pos, map);

	//Stop Lines
	std::cout << " >> Extract Stop lines data ... " << std::endl;
	ExtractStopLinesDataV2(mapRaw.pStopLines->m_data_list, mapRaw.pLines , mapRaw.pPoints , origin.pos, map);

	if(_load_lines)
	{
		//Lines
		std::cout << " >> Extract lines data ... " << std::endl;
		ExtractLines(mapRaw.pLines, mapRaw.pWhitelines, mapRaw.pPoints, origin.pos, map);
	}

	if(_load_curbs)
	{
		//Curbs
		std::cout << " >> Extract curbs data ... " << std::endl;
		ExtractCurbDataV2(mapRaw.pCurbs->m_data_list, mapRaw.pLines, mapRaw.pPoints, origin.pos, map);
	}

	if(_load_wayareas)
	{
		//Wayarea
		std::cout << " >> Extract wayarea data ... " << std::endl;
		ExtractWayArea(mapRaw.pAreas->m_data_list, mapRaw.pWayAreas->m_data_list, mapRaw.pLines->m_data_list, mapRaw.pPoints->m_data_list, origin.pos, map);
	}

	std::cout << " >> Connect Wayarea (boundaries) to waypoints ... " << std::endl;
	MappingHelpers::ConnectBoundariesToWayPoints(map);
	MappingHelpers::LinkBoundariesToWayPoints(map);

	//Link waypoints
	std::cout << " >> Link missing branches and waypoints... " << std::endl;
	MappingHelpers::LinkMissingBranchingWayPointsV2(map);

	//Link StopLines and Traffic Lights
	std::cout << " >> Link StopLines and Traffic Lights ... " << std::endl;
	MappingHelpers::LinkTrafficLightsAndStopLinesV2(map);
//	MappingHelpers::LinkTrafficLightsIntoGroups(map); // will use pole ID as group ID
	MappingHelpers::ConnectTrafficLightsAndStopLines(map);
	MappingHelpers::ConnectTrafficSignsAndStopLines(map);

	std::cout << " >> Map loaded from data with " << roadLanes.size()  << " lanes, and " << map.lines.size() << " lines." << std::endl;
}

void VectorMapLoader::CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, std::vector<PlannerHNS::Lane>& out_lanes)
{

	out_lanes.clear();
	std::vector<int> start_lines;
	GetLanesStartPoints(pLaneData, start_lines);
	for(unsigned int l =0; l < start_lines.size(); l++)
	{
		Lane _lane;
		GetLanePoints(pLaneData, pPointsData, pNodesData, start_lines.at(l), _lane);
		out_lanes.push_back(_lane);
	}
}

void VectorMapLoader::GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData,
				std::vector<int>& m_LanesStartIds)
{
	m_LanesStartIds.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	for(unsigned int il=0; il < pLaneData->m_data_list.size(); il++)
	{
		pL = &pLaneData->m_data_list.at(il);

		if(IsStartLanePoint(pLaneData, pL) == true)
		{
			m_LanesStartIds.push_back(pL->LnID);
		}
	}
}

bool VectorMapLoader::IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	//if(pL->JCT > 0 || pL->BLID == 0)
	if(pL->BLID == 0)
		return true;

	if(pL->BLID2 != 0 || pL->BLID3 != 0 || pL->BLID4 != 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pPrevL = pLaneData->GetDataRowById(pL->BLID);
	if(pPrevL == nullptr || pPrevL->FLID2 > 0 || pPrevL->FLID3 > 0 || pPrevL->FLID4 > 0 || pPrevL->JCT > 0)
		return true;

	return false;
}

bool VectorMapLoader::IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	if(pL->FLID2 > 0 || pL->FLID3 > 0 || pL->FLID4 > 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pNextL = pLaneData->GetDataRowById(pL->FLID);

	return IsStartLanePoint(pLaneData, pNextL);
}

void VectorMapLoader::GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, int lnID,
			PlannerHNS::Lane& out_lane)
{
	int next_lnid = lnID;
	bool bStart = false;
	bool bLast = false;
	int _rID = 0;
	out_lane.points.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	out_lane.id = lnID;
	out_lane.speed = 0;

	while(!bStart)
	{
		pL = pLaneData->GetDataRowById(next_lnid);
		if(pL == nullptr)
		{
			std::cout << "## Error, Can't find lane from ID: " << next_lnid << std::endl;
			break;
		}

		next_lnid = pL->FLID;
		if(next_lnid == 0)
		{
			bStart = true;
		}
		else
		{
			UtilityHNS::AisanLanesFileReader::AisanLane* pTempLanePointer = pLaneData->GetDataRowById(next_lnid);
			if(pTempLanePointer == nullptr)
			{
				std::cout << "Can't Find Lane:" << next_lnid << ", from Previous Lane = " << pL->LnID;
				return;
			}
			bStart = IsStartLanePoint(pLaneData, pTempLanePointer);
		}

//		if(_lnid == 1267 ) //|| _lnid == 1268 || _lnid == 1269 || _lnid == 958)
//			out_lane.id = lnID;

		if(out_lane.points.size() == 0)
		{
			WayPoint wp1;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->BNID)->PID, wp1);
			wp1.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp1.id = pL->BNID;
			wp1.originalMapID = pL->LnID;
			wp1.laneId = lnID;
			out_lane.speed = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;

			if(pL->BLID != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID2 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID2);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID3 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID3);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID4 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID4);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp1.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp1.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp1.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			WayPoint wp2;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->FNID)->PID, wp2);
			wp2.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp2.id = pL->FNID;
			wp2.originalMapID = pL->LnID;
			wp2.laneId = lnID;
			wp2.fromIds.push_back(wp1.id);

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp2.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp2.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp2.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			wp1.toIds.push_back(wp2.id);
			out_lane.points.push_back(wp1);
			out_lane.points.push_back(wp2);

		}
		else
		{
			WayPoint wp;
			UtilityHNS::AisanNodesFileReader::AisanNode* pTempNode = pNodesData->GetDataRowById(pL->FNID);
			if(pTempNode == nullptr)
			{
				std::cout << "Can't Find Node from Lane = " << pL->LnID << ", with FNID = " << pL->FNID;
				return;
			}

			GetPointFromDataList(pPointsData, pTempNode->PID, wp);
			wp.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp.id = pL->FNID;

			out_lane.points.at(out_lane.points.size()-1).toIds.push_back(wp.id);
			wp.fromIds.push_back(out_lane.points.at(out_lane.points.size()-1).id);

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			wp.originalMapID = pL->LnID;
			wp.laneId = lnID;

			if(MappingHelpers::IsPointExist(wp, out_lane.points))
				bStart = true;
			else
				out_lane.points.push_back(wp);
		}

//		if(next_lnid == 0)
//			break;
	}
}

bool VectorMapLoader::GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp)
{
	if(pPointsData == nullptr) return false;

	UtilityHNS::AisanPointsFileReader::AisanPoints* pP =  pPointsData->GetDataRowById(pid);

	if(pP!=nullptr)
	{
		out_wp.id = pP->PID;
		out_wp.pos.x = pP->Ly + _origin.pos.x;
		out_wp.pos.y = pP->Bx + _origin.pos.y;
		out_wp.pos.z = pP->H + _origin.pos.z;
		out_wp.pos.lat = pP->B;
		out_wp.pos.lon = pP->L;
		out_wp.pos.alt = pP->H;
		MappingHelpers::correct_gps_coor(out_wp.pos.lat, out_wp.pos.lon);
		return true;
	}

	return false;
}

int VectorMapLoader::GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	if(pLaneData == nullptr) return false;
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->FNID;
	}

	return 0;
}

int VectorMapLoader::GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->BNID;
	}

	return 0;
}

void VectorMapLoader::ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<PlannerHNS::Lane>& lanes)
{
	for(unsigned int il = 0; il < lanes.size(); il++)
	{
		WayPoint fp = lanes.at(il).points.at(0);
		UtilityHNS::AisanLanesFileReader::AisanLane* pFirstL = pLaneData->GetDataRowById(fp.originalMapID);
		if(pFirstL!=nullptr)
		{
			if(pFirstL->BLID > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID);
			if(pFirstL->BLID2 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID2);
			if(pFirstL->BLID3 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID3);
			if(pFirstL->BLID4 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID4);
		}

		WayPoint ep = lanes.at(il).points.at(lanes.at(il).points.size()-1);
		UtilityHNS::AisanLanesFileReader::AisanLane* pEndL = pLaneData->GetDataRowById(ep.originalMapID);
		if(pEndL!=nullptr)
		{
			if(pEndL->FLID > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID);
			if(pEndL->FLID2 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID2);
			if(pEndL->FLID3 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID3);
			if(pEndL->FLID4 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID4);
		}
	}
}

void VectorMapLoader::ExtractSignalDataV2(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,	UtilityHNS::AisanPointsFileReader* pPointData,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int is=0; is< signal_data.size(); is++)
	{
		//if(signal_data.at(is).Type == 2)
		{
			TrafficLight tl;
			tl.id = signal_data.at(is).ID;
			tl.linkID = signal_data.at(is).LinkID;
			tl.stoppingDistance = 0;
			tl.groupID = signal_data.at(is).PLID;
			switch(signal_data.at(is).Type)
			{
			case 1:
				tl.lightType= RED_LIGHT;
				break;
			case 2:
				tl.lightType = GREEN_LIGHT;
				break;
			case 3:
				tl.lightType = YELLOW_LIGHT; //r = g = 1
				break;
			case 4:
				tl.lightType = CROSS_RED;
				break;
			case 5:
				tl.lightType = CROSS_GREEN;
				break;
			default:
				tl.lightType = UNKNOWN_LIGHT;
				break;
			}

			for(unsigned int iv = 0; iv < vector_data.size(); iv++)
			{
				if(signal_data.at(is).VID == vector_data.at(iv).VID)
				{
					tl.horizontal_angle = -vector_data.at(iv).Hang-180.0;
					tl.vertical_angle = vector_data.at(iv).Vang;
					WayPoint p;
					GetPointFromDataList(pPointData, vector_data.at(iv).PID, p);
					p.pos.a = tl.horizontal_angle*DEG2RAD;
					tl.pose = p;
				}
			}
			map.trafficLights.push_back(tl);
			if(tl.id > RoadNetwork::g_max_traffic_light_id)
				RoadNetwork::g_max_traffic_light_id = tl.id;
		}
	}
}

void VectorMapLoader::ExtractStopLinesDataV2(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			UtilityHNS::AisanLinesFileReader* pLineData, UtilityHNS::AisanPointsFileReader* pPointData, const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ist=0; ist < stop_line_data.size(); ist++)
	{
		StopLine sl;
		sl.linkID = stop_line_data.at(ist).LinkID;
		sl.id = stop_line_data.at(ist).ID;
		if(stop_line_data.at(ist).TLID>0)
			sl.lightIds.push_back(stop_line_data.at(ist).TLID);
		else
			sl.stopSignID = 100+ist;

		UtilityHNS::AisanLinesFileReader::AisanLine* pLine = pLineData->GetDataRowById(stop_line_data.at(ist).LID);
		if(pLine != nullptr)
		{
			WayPoint p1, p2;

			if(GetPointFromDataList(pPointData, pLine->BPID, p1))
				sl.points.push_back(p1);

			if(GetPointFromDataList(pPointData, pLine->FPID, p2))
				sl.points.push_back(p2);
		}

		map.stopLines.push_back(sl);
		if(sl.id > RoadNetwork::g_max_stop_line_id)
			RoadNetwork::g_max_stop_line_id = sl.id;
	}
}

void VectorMapLoader::ExtractLines(UtilityHNS::AisanLinesFileReader* pLineData, UtilityHNS::AisanWhitelinesFileReader* pWhitelineData,
		UtilityHNS::AisanPointsFileReader* pPointsData, const GPSPoint& origin, RoadNetwork& map)
{
	Line l;
	for(auto& line : pLineData->m_data_list)
	{
		LINE_TYPE l_type = GENERAL_LINE;
		MARKING_COLOR l_color =  MARK_COLOR_WHITE;
		double l_width = 0;
		int l_original_type = 0;
		bool bFound = false;
		for(auto& wl : pWhitelineData->m_data_list)
		{
			if(wl.LID == line.LID)
			{
				l_type = DEFAULT_WHITE_LINE;
				l_color =  MARK_COLOR_WHITE;
				l_width = wl.Width;
				l_original_type  = wl.type;
				bFound = true;
				break;
			}
		}

		if(!bFound)
		{
			continue;
		}

		if(line.BLID != 0 && line.FLID != 0) // old line
		{
			WayPoint p;
			if(GetPointFromDataList(pPointsData, line.BPID, p))
				l.points.push_back(p);
		}

		if(line.BLID == 0) // new line
		{
			l.points.clear();
			l.id = line.LID;
			l.color = l_color;
			l.original_type = l_original_type;
			l.type = l_type;
			l.width = l_width;

			WayPoint p;
			if(GetPointFromDataList(pPointsData, line.BPID, p))
				l.points.push_back(p);
		}

		if(line.FLID == 0) //insert and reset points
		{
			WayPoint p;
			if(GetPointFromDataList(pPointsData, line.FPID, p))
				l.points.push_back(p);

			if(l.id > RoadNetwork::g_max_line_id)
				RoadNetwork::g_max_line_id = l.id;

			map.lines.push_back(l);
		}
	}
}

void VectorMapLoader::ExtractCurbDataV2(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				UtilityHNS::AisanLinesFileReader* pLinedata, UtilityHNS::AisanPointsFileReader* pPointsData,
				const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ic=0; ic < curb_data.size(); ic++)
	{
		Curb c;
		c.id = curb_data.at(ic).ID;
		c.height = curb_data.at(ic).Height;
		c.width = curb_data.at(ic).Width;
		c.laneId = curb_data.at(ic).LinkID;

		UtilityHNS::AisanLinesFileReader::AisanLine* pSingleLine = pLinedata->GetDataRowById(curb_data.at(ic).LID);
		std::vector<int> ids;
		if(ExtractSingleLine(pLinedata, pPointsData, curb_data.at(ic).LID, c.points) == true)
		{
			map.curbs.push_back(c);
		}
	}
}

void VectorMapLoader::ExtractWayArea(const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
		const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,
			const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int iw=0; iw < wayarea_data.size(); iw ++)
	{
		Boundary bound;
		bound.id = wayarea_data.at(iw).ID;

		for(unsigned int ia=0; ia < area_data.size(); ia ++)
		{
			if(wayarea_data.at(iw).AID == area_data.at(ia).AID)
			{
				int s_id = area_data.at(ia).SLID;
				int e_id = area_data.at(ia).ELID;

				for(unsigned int il=0; il< line_data.size(); il++)
				{
					if(line_data.at(il).LID >= s_id && line_data.at(il).LID <= e_id)
					{
						for(unsigned int ip=0; ip < points_data.size(); ip++)
						{
							if(points_data.at(ip).PID == line_data.at(il).BPID)
							{
								WayPoint p(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0);
								p.pos.lat = points_data.at(ip).B;
								p.pos.lon = points_data.at(ip).L;
								p.pos.alt = points_data.at(ip).H;
								MappingHelpers::correct_gps_coor(p.pos.lat, p.pos.lon);
								bound.points.push_back(p);
							}
						}
					}
				}
			}
		}

		map.boundaries.push_back(bound);
	}
}

void VectorMapLoader::ConstructRoadNetworkFromROSMessageVer0(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map)
{
	std::vector<Lane> roadLanes;
	Lane lane_obj;
	int laneIDSeq = 0;
	WayPoint prevWayPoint;
	UtilityHNS::AisanLanesFileReader::AisanLane prev_lane_point;
	UtilityHNS::AisanLanesFileReader::AisanLane curr_lane_point;
	UtilityHNS::AisanLanesFileReader::AisanLane next_lane_point;
	std::vector<std::pair<int,int> > id_replace_list;

	std::cout << std::endl << "Start Map Parsing, Vector Map Old Version 0" << std::endl;

	for(unsigned int l= 0; l < mapRaw.pLanes->m_data_list.size(); l++)
	{
		curr_lane_point = mapRaw.pLanes->m_data_list.at(l);
		curr_lane_point.originalMapID = -1;

		if(l+1 < mapRaw.pLanes->m_data_list.size())
		{
			next_lane_point = mapRaw.pLanes->m_data_list.at(l+1);
			if(curr_lane_point.FLID == next_lane_point.LnID && curr_lane_point.DID == next_lane_point.DID)
			{
				next_lane_point.BLID = curr_lane_point.BLID;
				if(next_lane_point.LaneDir == 'F')
					next_lane_point.LaneDir = curr_lane_point.LaneDir;

				if(curr_lane_point.BLID2 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID2;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID2;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID2;
				}

				if(curr_lane_point.BLID3 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID3;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID3;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID3;
				}

				if(curr_lane_point.BLID3 != 0)
				{
					if(next_lane_point.BLID2 == 0)	next_lane_point.BLID2 = curr_lane_point.BLID4;
					else if(next_lane_point.BLID3 == 0)	next_lane_point.BLID3 = curr_lane_point.BLID4;
					else if(next_lane_point.BLID4 == 0)	next_lane_point.BLID4 = curr_lane_point.BLID4;
				}

				if(curr_lane_point.FLID2 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID2;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID2;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID2;
				}

				if(curr_lane_point.FLID3 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID3;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID3;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID3;
				}

				if(curr_lane_point.FLID3 != 0)
				{
					if(next_lane_point.FLID2 == 0)	next_lane_point.FLID2 = curr_lane_point.FLID4;
					else if(next_lane_point.FLID3 == 0)	next_lane_point.FLID3 = curr_lane_point.FLID4;
					else if(next_lane_point.FLID4 == 0)	next_lane_point.FLID4 = curr_lane_point.FLID4;
				}

				if(prev_lane_point.FLID == curr_lane_point.LnID)
					prev_lane_point.FLID = next_lane_point.LnID;

				id_replace_list.push_back(std::make_pair(curr_lane_point.LnID, next_lane_point.LnID));
				int originalMapID = curr_lane_point.LnID;
				curr_lane_point = next_lane_point;
				curr_lane_point.originalMapID = originalMapID;
				l++;
			}
		}

		if(curr_lane_point.LnID != prev_lane_point.FLID)
		{
			if(laneIDSeq != 0) //first lane
			{
				lane_obj.toIds.push_back(prev_lane_point.FLID);
				roadLanes.push_back(lane_obj);
//				if(lane_obj.points.size() <= 1)
//					prev_FLID = 0;
			}

			laneIDSeq++;
			lane_obj = Lane();
			lane_obj.speed = curr_lane_point.LimitVel;
			lane_obj.id = curr_lane_point.LnID;
			lane_obj.fromIds.push_back(curr_lane_point.BLID);
			lane_obj.roadId = laneIDSeq;
		}

		WayPoint wp;
		bool bFound = false;
		bFound = GetWayPoint(curr_lane_point.LnID, lane_obj.id, curr_lane_point.RefVel,curr_lane_point.DID, mapRaw.pPoints, mapRaw.pCenterLines, origin.pos, wp);

		wp.originalMapID = curr_lane_point.originalMapID;

		if(curr_lane_point.LaneDir == 'L')
		{
			wp.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			//std::cout << " Left Lane : " << curr_lane_point.LnID << std::endl ;
		}
		else  if(curr_lane_point.LaneDir == 'R')
		{
			wp.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			//std::cout << " Right Lane : " << curr_lane_point.LnID << std::endl ;
		}
		else
		{
			wp.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
		}

		wp.fromIds.push_back(curr_lane_point.BLID);
		wp.toIds.push_back(curr_lane_point.FLID);

		//if(curr_lane_point.JCT > 0)
		if(curr_lane_point.FLID2 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID2);
			wp.toIds.push_back(curr_lane_point.FLID2);
		}
		if(curr_lane_point.FLID3 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID3);
			wp.toIds.push_back(curr_lane_point.FLID3);
		}
		if(curr_lane_point.FLID4 > 0)
		{
			lane_obj.toIds.push_back(curr_lane_point.FLID4);
			wp.toIds.push_back(curr_lane_point.FLID4);
		}

		if(curr_lane_point.BLID2 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID2);
			wp.fromIds.push_back(curr_lane_point.BLID2);
		}
		if(curr_lane_point.BLID3 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID3);
			wp.fromIds.push_back(curr_lane_point.BLID3);
		}
		if(curr_lane_point.BLID4 > 0)
		{
			lane_obj.fromIds.push_back(curr_lane_point.BLID4);
			wp.fromIds.push_back(curr_lane_point.BLID4);
		}

		//if(prev_lane_point.DID == curr_lane_point.DID && curr_lane_point.LnID == prev_lane_point.FLID)
//		if(prevWayPoint.pos.x == wp.pos.x && prevWayPoint.pos.y == wp.pos.y)
//		{
//			//if((prev_lane_point.FLID2 != 0 && curr_lane_point.FLID2 != 0) || (prev_lane_point.FLID3 != 0 && curr_lane_point.FLID3 != 0) || (prev_lane_point.FLID4 != 0 && curr_lane_point.FLID4 != 0))
//			{
//				cout << "Prev WP, LnID: " << prev_lane_point.LnID << ",BLID: " << prev_lane_point.BLID << ",FLID: " << prev_lane_point.FLID << ",DID: " << prev_lane_point.DID
//						<< ", Begin: " << prev_lane_point.BLID2 << "," << prev_lane_point.BLID3 << "," << prev_lane_point.BLID4
//						<< ", End: " << prev_lane_point.FLID2 << "," << prev_lane_point.FLID3 << "," << prev_lane_point.FLID4 << ": " << prev_lane_point.LaneDir <<   endl;
//				cout << "Curr WP, LnID: " << curr_lane_point.LnID << ",BLID: " << curr_lane_point.BLID << ",FLID: " << curr_lane_point.FLID << ",DID: " << curr_lane_point.DID
//						<< ", Begin: " << curr_lane_point.BLID2 <<  "," << curr_lane_point.BLID3 <<  "," << curr_lane_point.BLID4
//						<< ", End: " << curr_lane_point.FLID2 <<  "," <<curr_lane_point.FLID3 <<  "," << curr_lane_point.FLID4 <<   ": " << curr_lane_point.LaneDir << endl << endl;
//			}
//		}

		if(bFound)
		{
			lane_obj.points.push_back(wp);
			prevWayPoint = wp;
		}
		else
		{
			std::cout << " Strange ! point is not in the map !! " << std::endl;
		}

		prev_lane_point = curr_lane_point;
	}

//	//delete first two lanes !!!!! Don't know why , you don't know why ! , these two line cost you a lot .. ! why why , works for toyota map , but not with moriyama
//	if(bSpecialFlag)
//	{
//		if(roadLanes.size() > 0)
//			roadLanes.erase(roadLanes.begin()+0);
//		if(roadLanes.size() > 0)
//			roadLanes.erase(roadLanes.begin()+0);
//	}

	roadLanes.push_back(lane_obj);

	std::cout << " >> Replace Ids for a list of " << id_replace_list.size() << std::endl;

	for(unsigned int i =0; i < roadLanes.size(); i++)
	{
		Lane* pL = &roadLanes.at(i);
		ReplaceMyID(pL->id, id_replace_list);

		for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
		{
			int id = ReplaceMyID(pL->fromIds.at(j), id_replace_list);
			if(id != -1)
				pL->fromIds.at(j) = id;
		}

		for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
		{
			int id = ReplaceMyID(pL->toIds.at(j), id_replace_list);
			if(id != -1)
				pL->toIds.at(j) = id;
		}

		for(unsigned int j = 0 ; j < pL->points.size(); j++)
		{
			ReplaceMyID(pL->points.at(j).id, id_replace_list);
			ReplaceMyID(pL->points.at(j).laneId, id_replace_list);

			for(unsigned int jj = 0 ; jj < pL->points.at(j).fromIds.size(); jj++)
			{
				int id = ReplaceMyID(pL->points.at(j).fromIds.at(jj), id_replace_list);
				if(id != -1)
					pL->points.at(j).fromIds.at(jj) = id;
			}

			for(unsigned int jj = 0 ; jj < pL->points.at(j).toIds.size(); jj++)
			{
				int id = ReplaceMyID(pL->points.at(j).toIds.at(jj), id_replace_list);
				if(id != -1)
					pL->points.at(j).toIds.at(jj) = id;
			}
		}
	}

	std::cout << " >> Link Lanes and Waypoints" << std::endl;
	//Link Lanes and lane's waypoints by pointers
	//For each lane, the previous code set the fromId as the id of the last waypoint of the previos lane.
	//here we fix that by finding from each fromID the corresponding point and replace the fromId by the LaneID associated with that point.
	for(unsigned int l= 0; l < roadLanes.size(); l++)
	{
		for(unsigned int fp = 0; fp< roadLanes.at(l).fromIds.size(); fp++)
		{
			roadLanes.at(l).fromIds.at(fp) = MappingHelpers::GetLaneIdByWaypointId(roadLanes.at(l).fromIds.at(fp), roadLanes);
		}

		for(unsigned int tp = 0; tp< roadLanes.at(l).toIds.size(); tp++)
		{
			roadLanes.at(l).toIds.at(tp) = MappingHelpers::GetLaneIdByWaypointId(roadLanes.at(l).toIds.at(tp), roadLanes);
		}

		double sum_a = 0;
		for(unsigned int j = 0 ; j < roadLanes.at(l).points.size(); j++)
		{
			sum_a += roadLanes.at(l).points.at(j).pos.a;
		}
		roadLanes.at(l).dir = sum_a/(double)roadLanes.at(l).points.size();
	}

	//map has one road segment
	RoadSegment roadSegment1;
	roadSegment1.id = 1;
	roadSegment1.Lanes = roadLanes;
	map.roadSegments.push_back(roadSegment1);

	std::cout << " >> Link Lanes and waypoint by pointers" << std::endl;
	//Link Lanes and lane's waypoints by pointers
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
			    if(pL->points.at(j).actionCost.size() > 0)
			      {
				  if(pL->points.at(j).actionCost.at(0).first == LEFT_TURN_ACTION)
				    {
				      MappingHelpers::AssignActionCostToLane(pL, LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST);
				      break;
				    }
				  else if(pL->points.at(j).actionCost.at(0).first == RIGHT_TURN_ACTION)
				    {
				      MappingHelpers::AssignActionCostToLane(pL, RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST);
				    break;

				    }
			      }
			}
		}
	}

	if(_find_parallel_lanes)
	{
		std::cout << " >> Extract Lane Change Information... " << std::endl;
		MappingHelpers::FindAdjacentLanes(map);
	}

	std::cout << " >> Extract Signals .. " << std::endl;
	//Extract Signals and StopLines
	// Signals
	ExtractSignalData(mapRaw.pSignals->m_data_list, mapRaw.pVectors->m_data_list, mapRaw.pPoints->m_data_list , origin.pos, map);


	std::cout << " >> Extract StopLines .. " << std::endl;
	//Stop Lines
	ExtractStopLinesData(mapRaw.pStopLines->m_data_list, mapRaw.pLines->m_data_list, mapRaw.pPoints->m_data_list, origin.pos, map);


	std::cout << " >> Linke Branches .. " << std::endl;
	//Link waypoints
	MappingHelpers::LinkMissingBranchingWayPoints(map);

	//Link StopLines and Traffic Lights
	MappingHelpers::LinkTrafficLightsAndStopLines(map);
	//LinkTrafficLightsAndStopLinesConData(conn_data, id_replace_list, map);

	if(_load_curbs)
	{
		//Curbs
		std::cout << " >> Extract Curbs .. " << std::endl;
		ExtractCurbData(mapRaw.pCurbs->m_data_list, mapRaw.pLines->m_data_list, mapRaw.pPoints->m_data_list, origin.pos, map);
	}

	if(_load_wayareas)
	{
		//Wayarea
		std::cout << " >> Extract Wayarea .. " << std::endl;
		ExtractWayArea(mapRaw.pAreas->m_data_list, mapRaw.pWayAreas->m_data_list, mapRaw.pLines->m_data_list, mapRaw.pPoints->m_data_list, origin.pos, map);
	}

	//Fix angle for lanes
	std::cout << " >> Fix waypoint direction .. " << std::endl;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);
		}
	}

	MappingHelpers::LinkTrafficLightsIntoGroups(map);

	std::cout << "Map loaded from data with " << roadLanes.size()  << " lanes" << std::endl;
}

bool VectorMapLoader::GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did, UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanCenterLinesFileReader* pDtData, const GPSPoint& origin, WayPoint& way_point)
{

	if(pDtData == nullptr || pPointsData == nullptr) return false;

	UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine* pDT = pDtData->GetDataRowById(did);
	if(pDT != nullptr)
	{
		WayPoint wp;
		if(GetPointFromDataList(pPointsData, pDT->PID, wp))
		{
			wp.id = id;
			wp.laneId = laneID;
			wp.v = refVel;
			wp.pos.dir = pDT->Dir;
			wp.pos.a = pDT->Dir;
			wp.iOriginalIndex = wp.id;
			way_point = wp;
			return true;
		}
	}
	else
	{
		std::cout << "Can't find center line dt point ! " << did <<  std::endl;
	}

	return false;
}

int VectorMapLoader::ReplaceMyID(int& id,const std::vector<std::pair<int,int> >& rep_list)
{
	for(unsigned int i=0; i < rep_list.size(); i++)
	{
		if(rep_list.at(i).first == id)
		{
			id = rep_list.at(i).second;
			return id;
		}
	}

	return -1;
}

void VectorMapLoader::ExtractSignalData(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
			const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data,	const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int is=0; is< signal_data.size(); is++)
	{
		if(signal_data.at(is).Type == 2)
		{
			TrafficLight tl;
			tl.id = signal_data.at(is).ID;
			tl.linkID = signal_data.at(is).LinkID;
			tl.stoppingDistance = 0;

			for(unsigned int iv = 0; iv < vector_data.size(); iv++)
			{
				if(signal_data.at(is).VID == vector_data.at(iv).VID)
				{
					for(unsigned int ip = 0; ip < points_data.size(); ip++)
					{
						if(vector_data.at(iv).PID == points_data.at(ip).PID)
						{
							WayPoint p(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, (-vector_data.at(iv).Hang-180.0)*DEG2RAD);
							p.pos.lat = points_data.at(ip).B;
							p.pos.lon = points_data.at(ip).L;
							p.pos.alt = points_data.at(ip).H;
							MappingHelpers::correct_gps_coor(p.pos.lat, p.pos.lon);
							tl.pose = p;
							break;
						}
					}
				}
			}
			map.trafficLights.push_back(tl);
			if(tl.id > RoadNetwork::g_max_traffic_light_id)
				RoadNetwork::g_max_traffic_light_id = tl.id;
		}
	}
}

void VectorMapLoader::ExtractStopLinesData(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
			const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ist=0; ist < stop_line_data.size(); ist++)
		{
		StopLine sl;
		sl.linkID = stop_line_data.at(ist).LinkID;
		sl.id = stop_line_data.at(ist).ID;
		if(stop_line_data.at(ist).TLID>0)
			sl.lightIds.push_back(stop_line_data.at(ist).TLID);
		else
			sl.stopSignID = 100+ist;

		for(unsigned int il=0; il < line_data.size(); il++)
		{
			if(stop_line_data.at(ist).LID == line_data.at(il).LID)
			{
				int s_id = line_data.at(il).BPID;
				int e_id = line_data.at(il).FPID;
				for(unsigned int ip = 0; ip < points_data.size(); ip++)
				{
					if(points_data.at(ip).PID == s_id || points_data.at(ip).PID == e_id)
					{
						WayPoint p(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0);
						p.pos.lat = points_data.at(ip).B;
						p.pos.lon = points_data.at(ip).L;
						p.pos.alt = points_data.at(ip).H;
						MappingHelpers::correct_gps_coor(p.pos.lat, p.pos.lon);
						sl.points.push_back(p);
					}
				}
			}
		}
		map.stopLines.push_back(sl);
		if(sl.id > RoadNetwork::g_max_stop_line_id)
			RoadNetwork::g_max_stop_line_id = sl.id;
	}
}

void VectorMapLoader::ExtractCurbData(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map)
{
	for(unsigned int ic=0; ic < curb_data.size(); ic++)
		{
			Curb c;
			c.id = curb_data.at(ic).ID;

			for(unsigned int il=0; il < line_data.size(); il++)
			{
				if(curb_data.at(ic).LID == line_data.at(il).LID)
				{
					int s_id = line_data.at(il).BPID;
					//int e_id = line_data.at(il).FPID;
					for(unsigned int ip = 0; ip < points_data.size(); ip++)
					{
						if(points_data.at(ip).PID == s_id)
						{
							WayPoint p(points_data.at(ip).Ly + origin.x, points_data.at(ip).Bx + origin.y, points_data.at(ip).H + origin.z, 0);
							p.pos.lat = points_data.at(ip).B;
							p.pos.lon = points_data.at(ip).L;
							p.pos.alt = points_data.at(ip).H;
							MappingHelpers::correct_gps_coor(p.pos.lat, p.pos.lon);
							c.points.push_back(p);
							WayPoint wp;
							wp = c.points.at(0);
							Lane* pLane = MappingHelpers::GetClosestLaneFromMap(wp, map, 5);
							if(pLane)
							{
								c.laneId = pLane->id;
								c.pLane = pLane;
							}
						}
					}
				}
			}
			map.curbs.push_back(c);
		}
}

bool VectorMapLoader::ExtractSingleLine(UtilityHNS::AisanLinesFileReader* pLinedata, UtilityHNS::AisanPointsFileReader* pPointsData, const int& lid,
		std::vector<WayPoint>& out_line_points)
{
	out_line_points.clear();
	UtilityHNS::AisanLinesFileReader::AisanLine* pSingleLine = pLinedata->GetDataRowById(lid);
	if(pSingleLine->BLID != 0)
		return false;

	while(pSingleLine != nullptr)
	{

		WayPoint p1, p2;
		if(GetPointFromDataList(pPointsData, pSingleLine->BPID, p1) && GetPointFromDataList(pPointsData, pSingleLine->FPID, p2))
		{
			out_line_points.push_back(p1);

			if(pSingleLine->FLID == 0)
			{
				out_line_points.push_back(p2);
				return true;
			}

			pSingleLine = pLinedata->GetDataRowById(pSingleLine->FLID);
		}
		else
		{
			return false;
		}
	}

	if(out_line_points.size() == 0)
		return false;
	else
		return true;
}

void VectorMapLoader::GenerateDtLaneAndFixLaneForVectorMap(UtilityHNS::AisanLanesFileReader* pLaneData,
		UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData, PlannerHNS::RoadNetwork& map,
		std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtlane_data)
{
	int iDID = 0;

	std::vector<Lane> roadLanes;
	CreateLanes(pLaneData, pPointsData, pNodesData, roadLanes);
	MappingHelpers::FixTwoPointsLanes(roadLanes);
	MappingHelpers::FixRedundantPointsLanes(roadLanes);

	for(unsigned int il =0; il < roadLanes.size(); il++)
	{
		Lane* pL = &roadLanes.at(il);
		PlanningHelpers::SmoothPath(pL->points, 0.45, 0.3, 0.05);
		PlanningHelpers::SmoothPath(pL->points, 0.45, 0.3, 0.05);
		PlanningHelpers::CalcDtLaneInfo(pL->points);


		for(unsigned int ip =0; ip < pL->points.size(); ip++)
		{
			UtilityHNS::AisanLanesFileReader::AisanLane* pAL = nullptr;
			UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

			pAL = pLaneData->GetDataRowById(pL->points.at(ip).originalMapID);
			if(pAL != nullptr)
			{
				pN = pNodesData->GetDataRowById(pAL->BNID);
				if(pN != nullptr)
				{
					UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt_wp;
					dt_wp.DID = iDID++;
					dt_wp.PID = pN->PID;
					dt_wp.Dir = pL->points.at(ip).rot.z;
					dt_wp.Dist = pL->points.at(ip).cost;
					dt_wp.Apara = 0;
					dt_wp.LW = 0;
					dt_wp.RW = 0;
					dt_wp.cant = 0;
					dt_wp.r = pL->points.at(ip).rot.w;
					dt_wp.slope = pL->points.at(ip).rot.y;
					pAL->DID = dt_wp.DID;
					dtlane_data.push_back(dt_wp);
				}
			}
		}
	}
}

void VectorMapLoader::LinkTrafficLightsAndStopLinesConData(const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
		const std::vector<std::pair<int,int> >& id_replace_list, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{

			for(unsigned int ic = 0; ic < conn_data.size(); ic++)
			{
				UtilityHNS::AisanDataConnFileReader::DataConn data_conn = conn_data.at(ic);
				ReplaceMyID(data_conn.LID , id_replace_list);

				if(map.roadSegments.at(rs).Lanes.at(i).id == data_conn.LID)
				{
					for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
					{
						if(map.trafficLights.at(itl).id == data_conn.SID)
						{
							map.trafficLights.at(itl).laneIds.push_back(map.roadSegments.at(rs).Lanes.at(i).id);
							map.trafficLights.at(itl).pLanes.push_back(&map.roadSegments.at(rs).Lanes.at(i));
							map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						}
					}

					for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
					{
						if(map.stopLines.at(isl).id == data_conn.SLID)
						{
							map.stopLines.at(isl).laneId = map.roadSegments.at(rs).Lanes.at(i).id;
							map.stopLines.at(isl).pLane = &map.roadSegments.at(rs).Lanes.at(i);
							map.stopLines.at(isl).lightIds.push_back(data_conn.SID);
							map.stopLines.at(isl).stopSignID = data_conn.SSID;
							map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));
							WayPoint wp((map.stopLines.at(isl).points.at(0).pos.x+map.stopLines.at(isl).points.at(1).pos.x)/2.0, (map.stopLines.at(isl).points.at(0).pos.y+map.stopLines.at(isl).points.at(1).pos.y)/2.0, (map.stopLines.at(isl).points.at(0).pos.z+map.stopLines.at(isl).points.at(1).pos.z)/2.0, (map.stopLines.at(isl).points.at(0).pos.a+map.stopLines.at(isl).points.at(1).pos.a)/2.0);
							map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
						}
					}
				}
			}

		}
	}
}


} /* namespace PlannerHNS */
