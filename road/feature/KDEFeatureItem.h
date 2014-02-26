﻿#pragma once

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
	int id;
	QVector2D pt;
	float territory;
	std::vector<KDEFeatureItemEdge> edges;

public:
	KDEFeatureItem() : id(-1) {}
	KDEFeatureItem(int id) : id(id) {}
	~KDEFeatureItem() {}

	void addEdge(const Polyline2D &polyline, bool deadend);
	float getMinDistance(const Polyline2D &polyline) const;

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

