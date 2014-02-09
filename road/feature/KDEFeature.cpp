﻿#include <QDomDocument>
#include <QTextStream>
#include "KDEFeature.h"

KDEFeature::KDEFeature() : AbstractFeature() {
	_type = TYPE_GENERIC;
}

/**
 * 与えられたfeatureノード配下のXML情報に基づいて、グリッド特徴量を設定する。
 */
void KDEFeature::load(QDomNode& node) {
	items.clear();

	_weight = node.toElement().attribute("weight").toFloat();

	QDomNode child = node.firstChild();
	while (!child.isNull()) {
		if (child.toElement().tagName() == "center") {
			QDomNode child2 = child.firstChild();
			while (!child2.isNull()) {
				if (child2.toElement().tagName() == "x") {
					_center.setX(child2.firstChild().nodeValue().toFloat());
				} else if (child2.toElement().tagName() == "y") {
					_center.setY(child2.firstChild().nodeValue().toFloat());
				}

				child2 = child2.nextSibling();
			}
		} else if (child.toElement().tagName() == "item") {
			KDEFeatureItem item;
			item.load(child);
			items.push_back(item);
		}

		child = child.nextSibling();
	}
}

void KDEFeature::save(QDomDocument& doc, QDomNode& root) {
	QString str;

	str.setNum(_weight);
	QDomElement node_feature = doc.createElement("feature");
	node_feature.setAttribute("type", "generic");
	node_feature.setAttribute("weight", str);
	root.appendChild(node_feature);

	// write center node
	QDomElement node_center = doc.createElement("center");
	node_feature.appendChild(node_center);

	QDomElement node_center_x = doc.createElement("x");
	node_center.appendChild(node_center_x);

	str.setNum(_center.x());
	QDomText node_center_x_value = doc.createTextNode(str);
	node_center_x.appendChild(node_center_x_value);

	QDomElement node_center_y = doc.createElement("y");
	node_center.appendChild(node_center_y);

	str.setNum(_center.y());
	QDomText node_center_y_value = doc.createTextNode(str);
	node_center_y.appendChild(node_center_y_value);

	for (int i = 0; i < items.size(); ++i) {
		// write item node
		QDomElement node_item = doc.createElement("item");
		node_feature.appendChild(node_item);
		items[i].save(doc, node_item);
	}
}
