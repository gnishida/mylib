#pragma once

#include <vector>
#include <QVector2D>
#include <boost/shared_ptr.hpp>
#include "feature/KDEFeatureItem.h"

class RoadVertex {
public:
	QVector2D pt;
	bool valid;
	bool seed;

	bool onBoundary;
	std::vector<float> angles;
	std::vector<float> lengths;

	KDEFeatureItem kernel;
	bool snapped;

public:
	RoadVertex();
	RoadVertex(const QVector2D &pt);

	const QVector2D& getPt() const;
};

typedef boost::shared_ptr<RoadVertex> RoadVertexPtr;
