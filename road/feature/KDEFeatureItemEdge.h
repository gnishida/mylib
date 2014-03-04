#pragma once

#include "../../common/Polyline2D.h"

class KDEFeatureItemEdge {
public:
	Polyline2D edge;
	bool deadend;
	bool confident;

public:
	KDEFeatureItemEdge(const Polyline2D &edge, bool deadend) : edge(edge), deadend(deadend) {}
	~KDEFeatureItemEdge() {}

	void scale(float scaleX, float scaleY, const QVector2D &orig);
};

