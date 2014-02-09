#pragma once

#include "AbstractFeature.h"

class KDEFeature : public AbstractFeature {
public:
	KDEFeature();
	~KDEFeature() {}

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

