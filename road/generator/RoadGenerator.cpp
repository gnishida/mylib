﻿#include "../../common/Util.h"
#include "../../common/VoronoiUtil.h"
#include "../GraphUtil.h"
#include "RoadGenerator.h"
#include "GridRoadGenerator.h"
#include "RadialRoadGenerator.h"
#include "KDERoadGenerator.h"
#include "KDERoadGenerator2.h"
#include "GenericRoadGenerator.h"

/**
 * 道路網を生成する
 * 現在の実装では、特徴量のXMLファイル内に最初に記述されたエリアの特徴量だけを使用して、道路網を生成する。
 * つまり、複数のPlaceTypeには対応していない。
 * 将来的に、複数のPlaceTypeに対応させたい。
 */
void RoadGenerator::generateRoadNetwork(RoadGraph& roads, const Polygon2D &area, const RoadFeature& rf, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, bool addAvenuesOnBoundary, int numIterations, bool isGenerateLocalStreets) {
	//roadArea.roads.clear();

	if (rf.features.size() == 0) return;

	GridRoadGenerator rg1;
	RadialRoadGenerator rg2;
	GenericRoadGenerator rg4;

	switch (rf.features[0]->type()) {
	case AbstractFeature::TYPE_GRID:
		rg1.generateRoadNetwork(roads, area, dynamic_cast<GridFeature&>(*rf.features[0]));
		break;
	case AbstractFeature::TYPE_RADIAL:
		rg2.generateRoadNetwork(roads, area, dynamic_cast<RadialFeature&>(*rf.features[0]));
		break;
	case AbstractFeature::TYPE_KDE:
		KDERoadGenerator::generateRoadNetwork(roads, area, dynamic_cast<KDEFeature&>(*rf.features[0]), invadingCheck, weightEdge, weightLocation, weightRepetition, addAvenuesOnBoundary, numIterations, isGenerateLocalStreets);
		break;
	case AbstractFeature::TYPE_GENERIC:
		rg4.generateRoadNetwork(roads, area, dynamic_cast<GenericFeature&>(*rf.features[0]));
		break;
	}

	roads.setModified();
}

