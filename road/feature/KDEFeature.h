#pragma once

#include <vector>
#include <QDomNode>
#include <boost/shared_ptr.hpp>
#include "AbstractFeature.h"
#include "KDEFeatureItem.h"
#include "../../common/RoadEdge.h"

class KDEFeature : public AbstractFeature {
public:
	int group_id;
	float _avenueDensity;						// how many vertices in 1km x 1km area
	float _streetDensity;						// how many vertices in 1km x 1km area
	std::vector<KDEFeatureItem> avenueItems;
	std::vector<KDEFeatureItem> streetItems;

public:
	KDEFeature();
	KDEFeature(int group_id);
	~KDEFeature() {}

	void setDensity(int roadType, float density);
	void addItem(int roadType, const KDEFeatureItem &item);

	void load(QDomNode& node);
	void loadAvenue(QDomNode& node);
	void loadStreet(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
	void saveAvenue(QDomDocument& doc, QDomNode& node);
	void saveStreet(QDomDocument& doc, QDomNode& node);
};

typedef boost::shared_ptr<KDEFeature> KDEFeaturePtr;