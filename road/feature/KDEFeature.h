#pragma once

#include <vector>
#include <QDomNode>
#include <boost/shared_ptr.hpp>
#include "AbstractFeature.h"
#include "KDEFeatureItem.h"

class KDEFeature : public AbstractFeature {
public:
	int group_id;
	float _density;						// how many vertices in 1km x 1km area
	std::vector<KDEFeatureItem> items;

public:
	KDEFeature();
	KDEFeature(int group_id);
	~KDEFeature() {}

	void setDensity(float density);
	void addItem(const KDEFeatureItem &item);

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

typedef boost::shared_ptr<KDEFeature> KDEFeaturePtr;