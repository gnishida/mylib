#pragma once

#include <vector>
#include <road/feature/AbstractFeature.h>
#include <road/feature/GridFeature.h>
#include <road/feature/RadialFeature.h>
#include <road/feature/KDEFeature.h>
#include <road/feature/GenericFeature.h>

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

