#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include "KDEFeatureItem.h"

void KDEFeatureItem::addEdge(const Polyline2D &polyline, bool deadend) {
	edges.push_back(polyline);
	deadends.push_back(deadend);
}

/**
 * 与えられた方向・長さに最も近い距離を返却する。
 */
float KDEFeatureItem::getMinDistance(const Polyline2D &polyline) const {
	float min_dist2 = std::numeric_limits<float>::max();

	for (int i = 0; i < edges.size(); ++i) {
		QVector2D dir1 = edges[i][edges[i].size() - 1] - edges[i][0];
		QVector2D dir2 = polyline[polyline.size() - 1] - polyline[0];

		float dist2 = (dir1 - dir2).lengthSquared();
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
			Polyline2D polyline;

			QDomNode child2 = child.firstChild();
			while (!child2.isNull()) {
				if (child2.toElement().tagName() == "point") {
					float x = child2.toElement().attribute("x").toFloat();
					float y = child2.toElement().attribute("y").toFloat();
					polyline.push_back(QVector2D(x, y));
				}

				child2 = child2.nextSibling();
			}

			edges.push_back(polyline);

			if (child.toElement().attribute("deadend") == "true") {
				deadends.push_back(true);
			} else {
				deadends.push_back(false);
			}
		}

		child = child.nextSibling();
	}
}

void KDEFeatureItem::save(QDomDocument& doc, QDomNode& node) {
	for (int i = 0; i < edges.size(); ++i) {
		QDomElement node_edge = doc.createElement("edge");

		QString str;
		if (deadends[i]) {
			node_edge.setAttribute("deadend", "true");
		} else {
			node_edge.setAttribute("deadend", "false");
		}

		for (int j = 0; j < edges[i].size(); ++j) {
			QDomElement node_point = doc.createElement("point");

			str.setNum(edges[i][j].x());
			node_point.setAttribute("x", str);
			str.setNum(edges[i][j].y());
			node_point.setAttribute("y", str);

			node_edge.appendChild(node_point);
		}
		
		node.appendChild(node_edge);
	}
}

