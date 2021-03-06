﻿#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include "RoadFeature.h"

QString RoadFeature::version = "2014-02-26";

void RoadFeature::clear() {
	features.clear();
}

void RoadFeature::load(QString filename) {
	QFile file(filename);

	QDomDocument doc;
	doc.setContent(&file, true);
	QDomElement root = doc.documentElement();

	if (root.attribute("version") != version) {
		std::cerr << "ERROR: This feature file is generated by the older version of the tool." << std::endl;
		return;
	}

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "feature") {
			if (node.toElement().attribute("type") == "grid") {
				GridFeaturePtr gf = GridFeaturePtr(new GridFeature(features.size()));
				gf->load(node);
				features.push_back(gf);
			} else if (node.toElement().attribute("type") == "radial") {
				RadialFeaturePtr rf = RadialFeaturePtr(new RadialFeature(features.size()));
				rf->load(node);
				features.push_back(rf);
			} else if (node.toElement().attribute("type") == "kde") {
				KDEFeaturePtr kf = KDEFeaturePtr(new KDEFeature(features.size()));
				kf->load(node);
				features.push_back(kf);
			} else if (node.toElement().attribute("type") == "generic") {
				GenericFeaturePtr gf = GenericFeaturePtr(new GenericFeature(features.size()));
				gf->load(node);
				features.push_back(gf);
			}
		}

		node = node.nextSibling();
	}
}

void RoadFeature::save(QString filename) {
	QDomDocument doc;

	QDomElement root = doc.createElement("features");
	root.setAttribute("version", version);
	doc.appendChild(root);

	for (int i = 0; i < features.size(); ++i) {
		features[i]->save(doc, root);
	}

	// write the dom to the file
	QFile file(filename);
	file.open(QIODevice::WriteOnly);

	QTextStream out(&file);
	doc.save(out, 4);
}

void RoadFeature::addFeature(AbstractFeaturePtr feature) {
	features.push_back(feature);
}

/**
 * 抽出した特徴量を正規化する。
 * weightの正規化、centerの正規化を行う。
 */
void RoadFeature::normalize() {
	float total_weight = 0.0f;
	QVector2D total_center;

	// total weight、total centerを計算
	for (int i = 0; i < features.size(); ++i) {
		total_weight += features[i]->weight();
		total_center += features[i]->center();
	}

	total_center /= features.size();

	// total weight、total centerに基づいて、weightとcenterをnormalizeする
	for (int i = 0; i < features.size(); ++i) {
		features[i]->setWeight(features[i]->weight() / total_weight);
		features[i]->setCenter(features[i]->center() - total_center);
	}
}

void RoadFeature::rotate(float deg) {
	for (int i = 0; i < features.size(); ++i) {
		features[i]->rotate(deg);
	}
}

void RoadFeature::scale(const Polygon2D &area) {
	for (int i = 0; i < features.size(); ++i) {
		features[i]->scale(area);
	}
}