#pragma once

#include "RoadArea.h"

class RoadAreaSet {
public:
	std::vector<RoadArea> areas;

public:
	RoadAreaSet() {}
	~RoadAreaSet() {}

	const size_t size() const;
	RoadArea& operator[](int index);
	void add(const RoadArea &area);
	void clear();
	void remove(int index);

	void load(QString filename);
	void save(QString filename);
};

