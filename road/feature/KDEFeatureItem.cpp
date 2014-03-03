#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include "../../common/Util.h"
#include "KDEFeatureItem.h"

void KDEFeatureItem::addEdge(const Polyline2D &polyline, bool deadend) {
	edges.push_back(KDEFeatureItemEdge(polyline, deadend));
}

/**
 * 与えられた方向・長さに最も近い距離を返却する。
 * edges[i]は、頂点自身の座標である(0, 0)を含まず、次の点の座標から始まる。しかも、各点の座標は、頂点自身の座標からの相対座標である。
 * 一方、polylineには、頂点自身の座標から始まる、各点の絶対座標が入っている。
 * したがって、polylineに対しては、polyline[0]を引いて相対座標にしてやる必要がある。
 */
float KDEFeatureItem::getMinDistance(const Polyline2D &polyline) const {
	float min_dist2 = std::numeric_limits<float>::max();

	for (int i = 0; i < edges.size(); ++i) {
		if (edges[i].edge.size() == 0) continue;

		// 頂点から、エッジのもう一方の端へのベクトルを計算する
		QVector2D dir1 = edges[i].edge.last();
		QVector2D dir2 = polyline.last() - polyline[0];

		float dist2 = (dir1 - dir2).lengthSquared();
		if (dist2 < min_dist2) {
			min_dist2 = dist2;
		}
	}

	return sqrtf(min_dist2);
}

/**
 * 指定された角度[degree]だけ、交差点カーネルを時計回りに回転する。
 */
void KDEFeatureItem::rotate(float deg, const QVector2D &orig) {
	pt = Util::rotate(pt, -Util::deg2rad(deg), orig);

	for (int i = 0; i < edges.size(); ++i) {
		edges[i].edge.rotate(deg, orig);
	}
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

			edges.push_back(KDEFeatureItemEdge(polyline, child.toElement().attribute("deadend") == "true"));
		}

		child = child.nextSibling();
	}
}

void KDEFeatureItem::save(QDomDocument& doc, QDomNode& node) {
	for (int i = 0; i < edges.size(); ++i) {
		QDomElement node_edge = doc.createElement("edge");

		QString str;
		if (edges[i].deadend) {
			node_edge.setAttribute("deadend", "true");
		} else {
			node_edge.setAttribute("deadend", "false");
		}

		for (int j = 0; j < edges[i].edge.size(); ++j) {
			QDomElement node_point = doc.createElement("point");

			node_point.setAttribute("x", edges[i].edge[j].x());
			node_point.setAttribute("y", edges[i].edge[j].y());

			node_edge.appendChild(node_point);
		}
		
		node.appendChild(node_edge);
	}
}

