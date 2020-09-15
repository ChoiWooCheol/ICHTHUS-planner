/*
 * VectorMapLoader.h
 *
 *  Created on: Mar 16, 2020
 *      Author: hatem
 */

/// \file VectorMapLoader.h
/// \brief Functions for Loading AISAN vector maps and initialize RoadNetwork map for OpenPlanner.
/// 		It could load from folder consists of .csv files, or from messages published by vector map loader in autoware
/// \author Hatem Darweesh
/// \date Mar 16, 2020

#ifndef VECTORMAPLOADER_H_
#define VECTORMAPLOADER_H_

#include <op_planner/MappingHelpers.h>
#include <op_utility/DataRW.h>

namespace PlannerHNS {

class VectorMapLoader {
public:
	VectorMapLoader(int map_version = 1, bool enable_lane_change = false, bool load_curb = false, bool load_lines = false, bool load_wayarea = false);
	virtual ~VectorMapLoader();

	/**
	 * @brief Sometimes additional origin needed to align the map, currently default origin is zero
	 * @param origin
	 */
	void SetAdditionalOrigin(const PlannerHNS::WayPoint& origin);

	/**
	 * @brief Read .csv based vector map files format. It is main format used by Autoware framework, developed by AISAN Technologies.
	 * 			Only .csv files matching predefined item names will be loaded.
	 * @param vector map folder name
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadFromFile(const std::string& vectorMapFolder, PlannerHNS::RoadNetwork& map);

	/**
	 * @brief Convert vector map from data items raw data into OpenPlanner RoadNetwork format
	 * @param mapRaw, contains a list of all vector map items, which representing each .csv file.
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadFromData(UtilityHNS::MapRaw& mapRaw, PlannerHNS::RoadNetwork& map);

private:
	int _map_version;
	bool _find_parallel_lanes;
	bool _load_curbs;
	bool _load_lines;
	bool _load_wayareas;
	PlannerHNS::WayPoint _origin;

	void ConstructRoadNetworkFromROSMessage(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map);
	void ConstructRoadNetworkFromROSMessageVer0(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map);
	void CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, std::vector<PlannerHNS::Lane>& out_lanes);
	void ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<PlannerHNS::Lane>& lanes);
	void GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<int>& m_LanesStartIds);
	void GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, int lnID, PlannerHNS::Lane& out_lane);
	bool IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);
	bool IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);
	bool GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp);
	int GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, const int& LnID);
	int GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID);
	void ExtractSignalDataV2(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
				const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data, UtilityHNS::AisanPointsFileReader* pPointsData,
				const GPSPoint& origin, RoadNetwork& map);
	void ExtractStopLinesDataV2(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
					UtilityHNS::AisanLinesFileReader* pLineData, UtilityHNS::AisanPointsFileReader* pPointData, const GPSPoint& origin, RoadNetwork& map);
	void ExtractLines(UtilityHNS::AisanLinesFileReader* pLineData, UtilityHNS::AisanWhitelinesFileReader* pWhitelineData,
			UtilityHNS::AisanPointsFileReader* pPointsData, const GPSPoint& origin, RoadNetwork& map);
	void ExtractCurbDataV2(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data, UtilityHNS::AisanLinesFileReader* pLinedata,
					UtilityHNS::AisanPointsFileReader* pPointsData, const GPSPoint& origin, RoadNetwork& map);
	void ExtractWayArea(const std::vector<UtilityHNS::AisanAreasFileReader::AisanArea>& area_data,
			const std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea>& wayarea_data, const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
			const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map);
	bool GetWayPoint(const int& id, const int& laneID,const double& refVel, const int& did, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanCenterLinesFileReader* pDtData, const GPSPoint& origin, WayPoint& way_point);
	int ReplaceMyID(int& id, const std::vector<std::pair<int,int> >& rep_list);
	void ExtractSignalData(const std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal>& signal_data,
				const std::vector<UtilityHNS::AisanVectorFileReader::AisanVector>& vector_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map);
	void ExtractStopLinesData(const std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine>& stop_line_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map);
	void ExtractCurbData(const std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb>& curb_data,
				const std::vector<UtilityHNS::AisanLinesFileReader::AisanLine>& line_data,
				const std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints>& points_data, const GPSPoint& origin, RoadNetwork& map);

	/**
	 * @brief Extract line points of lid is the first point of the line, BLID has to be zero, otherwise the function will return 0 points
	 * @param pLine line data
	 * @param lid line row id in the line.csv vector map file
	 * @param out_line_points the points from BLID=0 to FLID=0
	 * @return true of line is extracted, false if lid row is not the beginning of the line (BLID != 0).
	 */
	bool ExtractSingleLine(UtilityHNS::AisanLinesFileReader* pLine, UtilityHNS::AisanPointsFileReader* pPoint, const int& lid,
			std::vector<WayPoint>& out_line_points);

	/**
	 * @brief Fix dt lane problem for vector maps generated from the free web based mapping service.
	 * @param pLaneData
	 * @param pPointsData
	 * @param pNodesData
	 * @param map
	 * @param dtlane_data
	 */
	void GenerateDtLaneAndFixLaneForVectorMap(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData,
			PlannerHNS::RoadNetwork& map, std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine>& dtlane_data);

	/**
	 * @brief Old data connection function, used for old vector map versions
	 * @param conn_data
	 * @param id_replace_list
	 * @param map
	 */
	void LinkTrafficLightsAndStopLinesConData(const std::vector<UtilityHNS::AisanDataConnFileReader::DataConn>& conn_data,
			const std::vector<std::pair<int,int> >& id_replace_list, RoadNetwork& map); //pointers link

};

} /* namespace PlannerHNS */

#endif /* VECTORMAPLOADER_H_ */
