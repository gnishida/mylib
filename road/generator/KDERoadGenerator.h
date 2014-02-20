#pragma once

#include "../../common/Polygon2D.h"
#include "../RoadGraph.h"
#include "../feature/KDEFeature.h"

class KDERoadGenerator {
public:
	KDERoadGenerator() {}
	~KDERoadGenerator() {}

	void generateRoadNetwork(RoadGraph &roads, Polygon2D &area, const KDEFeature& kf, int numIterations, bool isGenerateLocalStreets);

private:
	void generateBoulevard(RoadGraph &roads, Polygon2D &area);
	void generateAvenueSeeds(RoadGraph &roads, Polygon2D &area, const KDEFeature& f, std::list<RoadVertexDesc>& seeds);
	void addAvenueSeed(RoadGraph &roads, const Polygon2D &area, const KDEFeature &f, const QVector2D &offset, QSet<int> &usedKernels, std::list<RoadVertexDesc>& seeds);
	void generateStreetSeeds(RoadGraph &roads, const KDEFeature& f, std::list<RoadVertexDesc>& seeds);

	void attemptExpansion(RoadGraph &roads, Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, std::list<RoadVertexDesc> &seeds);
	bool growRoadSegment(RoadGraph &roads, Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, const KDEFeatureItemEdge &edge, std::list<RoadVertexDesc> &seeds);

	bool intersects(RoadGraph &roads, const QVector2D& p0, const QVector2D& p1, RoadEdgeDesc &eiClosest, QVector2D &closestIntPt);

	KDEFeatureItem getItem(RoadGraph &roads, const KDEFeature& kf, int roadType, RoadVertexDesc v_desc, const QVector2D &pt);

	bool canSnapToVertex(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc);
	bool canSnapToEdge(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt);


	float getNearestVertex(RoadGraph& roads, const QVector2D& pos, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc);
	float getNearestEdge(RoadGraph& roads, const QVector2D& pt, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt);
	void connectAvenues(RoadGraph &roads, float threshold);
};

