#include <QDomDocument>
#include <QTextStream>
#include "KDEFeature.h"

KDEFeature::KDEFeature() : AbstractFeature() {
	_type = TYPE_KDE;
}

KDEFeature::KDEFeature(int group_id) : AbstractFeature() {
	_type = TYPE_KDE;
	this->group_id = group_id;
}

void KDEFeature::setDensity(int roadType, float density) {
	switch (roadType) {
	case RoadEdge::TYPE_AVENUE:
		_avenueDensity = density;
		break;
	case RoadEdge::TYPE_STREET:
		_streetDensity = density;
		break;
	}
}

void KDEFeature::addItem(int roadType, const KDEFeatureItem &item) {
	switch (roadType) {
	case RoadEdge::TYPE_AVENUE:
		_avenueItems.push_back(item);
		break;
	case RoadEdge::TYPE_STREET:
		_streetItems.push_back(item);
		break;
	}
}

const std::vector<KDEFeatureItem>& KDEFeature::items(int roadType) const {
	switch (roadType) {
	case RoadEdge::TYPE_AVENUE:
		return _avenueItems;
	case RoadEdge::TYPE_STREET:
		return _streetItems;
	default:
		return _streetItems;
	}
}

/**
 * 与えられたfeatureノード配下のXML情報に基づいて、グリッド特徴量を設定する。
 */
void KDEFeature::load(QDomNode& node) {
	_avenueItems.clear();
	_streetItems.clear();

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
		} else if (child.toElement().tagName() == "avenue") {
			_avenueDensity = child.toElement().attribute("density").toFloat();
			loadAvenue(child);
		} else if (child.toElement().tagName() == "street") {
			_streetDensity = child.toElement().attribute("density").toFloat();
			loadStreet(child);
		}

		child = child.nextSibling();
	}
}

void KDEFeature::loadAvenue(QDomNode& node) {
	QDomNode child = node.firstChild();
	while (!child.isNull()) {
		if (child.toElement().tagName() == "item") {
			KDEFeatureItem item;
			item.load(child);
			_avenueItems.push_back(item);
		}

		child = child.nextSibling();
	}
}

void KDEFeature::loadStreet(QDomNode& node) {
	QDomNode child = node.firstChild();
	while (!child.isNull()) {
		if (child.toElement().tagName() == "item") {
			KDEFeatureItem item;
			item.load(child);
			_streetItems.push_back(item);
		}

		child = child.nextSibling();
	}
}

void KDEFeature::save(QDomDocument& doc, QDomNode& root) {
	QString str;

	str.setNum(_weight);
	QDomElement node_feature = doc.createElement("feature");
	node_feature.setAttribute("type", "kde");
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

	// write avenue items
	str.setNum(_avenueDensity);
	QDomElement node_avenue = doc.createElement("avenue");
	node_avenue.setAttribute("density", _avenueDensity);
	node_feature.appendChild(node_avenue);

	saveAvenue(doc, node_avenue);

	// write street items
	QDomElement node_street = doc.createElement("street");
	node_street.setAttribute("density", _streetDensity);
	node_feature.appendChild(node_street);

	saveStreet(doc, node_street);
}

void KDEFeature::saveAvenue(QDomDocument& doc, QDomNode& node) {
	for (int i = 0; i < _avenueItems.size(); ++i) {
		QDomElement node_item = doc.createElement("item");
		node.appendChild(node_item);
		_avenueItems[i].save(doc, node_item);
	}
}

void KDEFeature::saveStreet(QDomDocument& doc, QDomNode& node) {
	for (int i = 0; i < _streetItems.size(); ++i) {
		QDomElement node_item = doc.createElement("item");
		node.appendChild(node_item);
		_streetItems[i].save(doc, node_item);
	}
}
