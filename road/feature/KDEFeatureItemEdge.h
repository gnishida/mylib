#pragma once

#include "../../common/Polyline2D.h"

class KDEFeatureItemEdge {
public:
	Polyline2D edge;
	bool deadend;

public:
	KDEFeatureItemEdge(const Polyline2D &edge, bool deadend) : edge(edge), deadend(deadend) {}
	~KDEFeatureItemEdge() {}
};

