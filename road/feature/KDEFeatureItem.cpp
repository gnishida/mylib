#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include "KDEFeatureItem.h"

void KDEFeatureItem::addEdge(float angle, float length) {
	angles.push_back(angle);
	lengths.push_back(length);
}

void KDEFeatureItem::load(QDomNode& node) {
	angles.clear();
	lengths.clear();

	QDomNode child = node.firstChild();
	while (!child.isNull()) {
		if (child.toElement().tagName() == "edge") {
			float angle = child.toElement().attribute("angle").toFloat();
			float length = child.toElement().attribute("length").toFloat();

			angles.push_back(angle);
			lengths.push_back(length);
		}

		child = child.nextSibling();
	}
}

void KDEFeatureItem::save(QDomDocument& doc, QDomNode& node) {
	// write edge node
}

