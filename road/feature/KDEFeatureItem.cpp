#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include "KDEFeatureItem.h"

void KDEFeatureItem::addEdge(const QVector2D &edge) {
	edges.push_back(edge);
}

/**
 * 与えられた方向・長さに最も近い距離を返却する。
 */
float KDEFeatureItem::getMinDistance(const QVector2D &edge) const {
	float min_dist2 = std::numeric_limits<float>::max();

	for (int i = 0; i < edges.size(); ++i) {
		float dist2 = (edges[i] - edge).lengthSquared();
		if (dist2 < min_dist2) {
			min_dist2 = dist2;
		}
	}

	return sqrtf(min_dist2);
}

void KDEFeatureItem::load(QDomNode& node) {
	edges.clear();

	QDomNode child = node.firstChild();
	while (!child.isNull()) {
		if (child.toElement().tagName() == "edge") {
			float x = child.toElement().attribute("x").toFloat();
			float y = child.toElement().attribute("y").toFloat();

			edges.push_back(QVector2D(x, y));
		}

		child = child.nextSibling();
	}
}

void KDEFeatureItem::save(QDomDocument& doc, QDomNode& node) {
	for (int i = 0; i < edges.size(); ++i) {
		QDomElement node_edge = doc.createElement("edge");
		QString str;
		str.setNum(edges[i].x());
		node_edge.setAttribute("x", str);
		str.setNum(edges[i].y());
		node_edge.setAttribute("y", str);
		node.appendChild(node_edge);
	}
}

