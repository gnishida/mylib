#pragma once

#include "../../common/Polygon2D.h"
#include "../RoadGraph.h"
#include "../RoadAreaSet.h"
#include "../feature/KDEFeature.h"

class KDERoadGenerator2 {
public:
	KDERoadGenerator2() {}
	~KDERoadGenerator2() {}

	static void generateRoadNetwork(RoadGraph &roads, const Polygon2D &area, const KDEFeature& kf, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, bool addAvenuesOnBoundary, int numIterations, bool isGenerateLocalStreets);

private:
	static void generateRoadsOnBoundary(RoadGraph &roads, const Polygon2D &area, int roadType, int lanes);
	static void generateAvenueSeeds(RoadGraph &roads, const Polygon2D &area, const KDEFeature& f, std::list<RoadVertexDesc>& seeds);
	static void addAvenueSeed(RoadGraph &roads, const const Polygon2D &area, const KDEFeature &f, const QVector2D &offset, QSet<int> &usedKernels, std::list<RoadVertexDesc>& seeds);
	static void generateStreetSeeds(RoadGraph &roads, const Polygon2D &area, const KDEFeature& f, std::list<RoadVertexDesc>& seeds);

	static void attemptExpansion(RoadGraph &roads, const Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, std::list<RoadVertexDesc> &seeds);
	static bool growRoadSegment(RoadGraph &roads, const Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, const KDEFeatureItemEdge &edge, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, std::list<RoadVertexDesc> &seeds);

	static KDEFeatureItem getItem(RoadGraph &roads, const Polygon2D &area, const KDEFeature& kf, int roadType, RoadVertexDesc v_desc, float weightEdge, float weightLocation, float weightRepetition);
	static KDEFeatureItem getItem2(RoadGraph &roads, const Polygon2D &area, const KDEFeature& kf, int roadType, RoadVertexDesc v_desc, float weightEdge, float weightLocation, float weightRepetition);

	static void connectAvenues(RoadGraph &roads, float threshold);
	
public:
	static void connectRoads(RoadGraph &roads, RoadAreaSet &areas, float dist_threshold, float angle_threshold);
	static bool growRoadOneStep(RoadGraph& roads, RoadVertexDesc srcDesc, const QVector2D& step);
	static void connectRoads2(RoadAreaSet &areas, float dist_threshold, float angle_threshold);
	static void connectRoads2(RoadAreaSet &areas, int area_id, RoadVertexDesc v_desc, float dist_threshold, float angle_threshold);
};

