#pragma once

#include <vector>
#include "AbstractFeature.h"
#include "GridFeature.h"
#include "RadialFeature.h"
#include "KDEFeature.h"
#include "GenericFeature.h"

class RoadFeature {
public:
	std::vector<AbstractFeaturePtr> features;

public:
	RoadFeature() {}
	~RoadFeature() {}

	void clear();
	void load(QString filename);
	void save(QString filename);

	void addFeature(AbstractFeaturePtr feature);

	void normalize();
};

