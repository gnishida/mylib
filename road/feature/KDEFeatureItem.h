﻿#pragma once

#include <QVector2D>
#include <QDomNode>

/**
 * １つの頂点から出るエッジの特徴を表すクラス。
 */

#include <vector>

class KDEFeatureItem {
public:
	std::vector<QVector2D> edges;
	std::vector<bool> deadends;

public:
	KDEFeatureItem() {}
	~KDEFeatureItem() {}

	void addEdge(const QVector2D &edge, bool deadend);
	float getMinDistance(const QVector2D &edge) const;

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

