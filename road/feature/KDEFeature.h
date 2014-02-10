#pragma once

#include <vector>
#include <QDomNode>
#include <boost/shared_ptr.hpp>
#include "AbstractFeature.h"
#include "KDEFeatureItem.h"

class KDEFeature : public AbstractFeature {
public:
	int group_id;
	std::vector<KDEFeatureItem> items;

public:
	KDEFeature();
	KDEFeature(int group_id);
	~KDEFeature() {}

	void addItem(const KDEFeatureItem &item);

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

typedef boost::shared_ptr<KDEFeature> KDEFeaturePtr;