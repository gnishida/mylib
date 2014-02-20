#pragma once

#include "../../common/Polygon2D.h"
#include "../RoadGraph.h"
#include "../RoadArea.h"
#include "../feature/RoadFeature.h"

class RoadGenerator {
public:
	RoadGenerator() {}
	~RoadGenerator() {}

	void generateRoadNetwork(RoadArea& roadArea, const RoadFeature& rf, int numIterations = 0, bool isGenerateLocalStreets = true);

private:
};

