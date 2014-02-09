#pragma once

#include <QDomNode>

/**
 * １つの頂点から出るエッジの特徴を表すクラス。
 */

#include <vector>

class KDEFeatureItem {
public:
	std::vector<float> angles;
	std::vector<float> lengths;

public:
	KDEFeatureItem() {}
	~KDEFeatureItem() {}

	void addEdge(float angle, float length);

	void load(QDomNode& node);
	void save(QDomDocument& doc, QDomNode& node);
};

