#pragma once

#include <vector>
#include <road/GridFeature.h>
#include <road/RadialFeature.h>
#include <road/GenericFeature.h>
#include <road/AbstractFeature.h>

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

