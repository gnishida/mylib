#pragma once

#include "../../common/Polygon2D.h"
#include "../RoadGraph.h"
#include "../RoadArea.h"
#include "../feature/RoadFeature.h"

class RoadGenerator {
public:
	RoadGenerator() {}
	~RoadGenerator() {}

	void generateRoadNetwork(RoadGraph& roads, const Polygon2D &area, const RoadFeature& rf, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, bool addAvenuesOnBoundary = false, int numIterations = 0, bool isGenerateLocalStreets = true);

private:
};

