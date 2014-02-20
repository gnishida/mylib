#pragma once

#include "../RoadGraph.h"
#include "RoadFeature.h"
#include "KDEFeature.h"

class KDEFeatureExtractor {
public:
	KDEFeatureExtractor() {}
	~KDEFeatureExtractor() {}

	static void extractFeature(RoadGraph& roads, Polygon2D& area, RoadFeature& roadFeature);
};

