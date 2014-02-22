#pragma once

#include "../common/Polygon2D.h"
#include "RoadGraph.h"

class RoadArea {
public:
	Polygon2D area;
	RoadGraph roads;

public:
	RoadArea();
	RoadArea(const Polygon2D &area);
	~RoadArea();

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

