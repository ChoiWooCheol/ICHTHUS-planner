
/// \file KmlMapLoader.h
/// \brief Functions for Loading OpenPlanner .kml map file format.
/// \author Hatem Darweesh
/// \date Mar 16, 2020

#ifndef KMLMAPLOADER_H_
#define KMLMAPLOADER_H_

#include <op_planner/MappingHelpers.h>

namespace PlannerHNS {

class KmlMapLoader {
public:
	KmlMapLoader(int map_version = 1);
	virtual ~KmlMapLoader();

	/**
	 * @brief Read xml based .kml map file format. it is custom format for OpenPlanner internal road network map requirements
	 * @param kmlFile
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadKML(const std::string& kmlFile, RoadNetwork& map);

	void LoadKMLItems(const std::string& kmlFile, RoadNetwork& original_map, std::vector<Lane>& lanes, std::vector<Line>& lines,
			std::vector<StopLine>& stopLines, std::vector<Boundary>& boundaries, std::vector<Curb>& curbs,
			std::vector<Crossing>& crossings, std::vector<TrafficLight>& trafficLights, std::vector<TrafficSign>& signs,
			std::vector<Marking>& markings);

	/**
	 * @brief Should be called before LoadKML to take effect. stitching process should be withing the loading process , shouldn't be executed separately
	 */
	void EnableLaneStitching()
	{
		_bLaneStitch = true;
	}

	static TiXmlElement* GetHeadElement(TiXmlElement* pMainElem);
	static std::vector<int> GetIDsFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	static std::pair<ACTION_TYPE, double> GetActionPairFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	static std::vector<std::string> SplitString(const std::string& str, const std::string& token);

private:
	PlannerHNS::RoadNetwork* _pMap;
	int _map_version;
	bool _bLaneStitch;
	TiXmlElement* GetDataFolder(const std::string& folderName, TiXmlElement* pMainElem);
	std::vector<Line> GetLinesList(TiXmlElement* pElem);
	std::vector<Curb> GetCurbsList(TiXmlElement* pElem);
	std::vector<Boundary> GetBoundariesList(TiXmlElement* pElem);
	std::vector<Marking> GetMarkingsList(TiXmlElement* pElem);
	std::vector<Crossing> GetCrossingsList(TiXmlElement* pElem);
	std::vector<TrafficSign> GetTrafficSignsList(TiXmlElement* pElem);
	std::vector<TrafficLight> GetTrafficLightsList(TiXmlElement* pElem);
	std::vector<StopLine> GetStopLinesList(TiXmlElement* pElem);
	std::vector<Lane> GetLanesList(TiXmlElement* pElem);
	std::vector<RoadSegment> GetRoadSegmentsList(TiXmlElement* pElem);
	std::vector<std::string> GetStringsFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	std::vector<double> GetDoubleFromPrefix(const std::string& str, const std::string& prefix, const std::string& postfix);
	std::vector<WayPoint> GetCenterLaneData(TiXmlElement* pElem, const int& currLaneID);
	std::vector<WayPoint> GetCenterLaneDataVer0(TiXmlElement* pElem, const int& currLaneID);
	std::vector<WayPoint> GetWaypointsData(TiXmlElement* pElem);

};

} /* namespace PlannerHNS */

#endif /* KMLMAPLOADER_H_ */
