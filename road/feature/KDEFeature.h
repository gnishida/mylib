#pragma once

#include <vector>
#include <QDomNode>
#include "AbstractFeature.h"
#include "KDEFeatureItem.h"

class KDEFeature : public AbstractFeature {
public:
	std::vector<KDEFeatureItem> items;

public:
	KDEFeature();
	~KDEFeature() {}

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

