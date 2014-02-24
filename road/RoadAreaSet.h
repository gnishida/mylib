#pragma once

#include "RoadArea.h"

class RoadAreaSet {
public:
	RoadGraph roads;
	std::vector<RoadArea> areas;

public:
	RoadAreaSet() {}
	~RoadAreaSet() {}

	const size_t size() const;
	RoadArea& operator[](int index);
	void add(const RoadArea &area);
	void clear();
	void remove(int index);

	void setZ(float z);

	void addRoads(int roadType, int lanes, bool oneWay, const Polyline2D &polyline);
	void mergeRoads();

	void load(QString filename);
	void save(QString filename);
};

