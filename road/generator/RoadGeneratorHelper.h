#pragma once

#include "../RoadGraph.h"

/**
 * 道路網生成のヘルパークラス。
 * 道路網生成に必要な、汎用的な関数などを実装する。
 */
class RoadGeneratorHelper {
protected:
	RoadGeneratorHelper() {}
	~RoadGeneratorHelper() {}

public:
	static bool intersects(RoadGraph &roads, const QVector2D& p0, const QVector2D& p1, RoadEdgeDesc &eiClosest, QVector2D &closestIntPt);
	static bool canSnapToVertex(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc);
	static bool canSnapToEdge(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt);

	static float getNearestVertex(RoadGraph& roads, const QVector2D& pos, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc);
	static float getNearestEdge(RoadGraph& roads, const QVector2D& pt, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt);

	static bool withinTerritory(RoadGraph &roads, const QVector2D &pt, RoadVertexDesc ignore);


};

