#pragma once

#include <QVector2D>
#include <QDomNode>
#include "../../common/Polyline2D.h"
#include "KDEFeatureItemEdge.h"

/**
 * １つの頂点から出るエッジの特徴を表すクラス。
 */

#include <vector>

class KDEFeatureItem {
public:
	QVector2D pt;
	std::vector<KDEFeatureItemEdge> edges;

public:
	KDEFeatureItem() {}
	~KDEFeatureItem() {}

	void addEdge(const Polyline2D &polyline, bool deadend);
	float getMinDistance(const Polyline2D &polyline) const;

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

