#pragma once

#include <vector>
#include <road/feature/GridFeature.h>
#include <road/feature/RadialFeature.h>
#include <road/feature/GenericFeature.h>
#include <road/feature/AbstractFeature.h>

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

