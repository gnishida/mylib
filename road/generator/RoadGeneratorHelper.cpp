#include <limits>
#include "../../common/Util.h"
#include "../GraphUtil.h"
#include "RoadGeneratorHelper.h"

/**
* Checks if new edge will intersect an existing edge
**/
bool RoadGeneratorHelper::intersects(RoadGraph &roads, const QVector2D& p0, const QVector2D& p1, RoadEdgeDesc &eiClosest, QVector2D &closestIntPt) {
	float min_dist = std::numeric_limits<float>::max();
	bool intersected = false;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// Highwayとはintersectしない
		if (roads.graph[*ei]->type == RoadEdge::TYPE_HIGHWAY) continue;

		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; ++i) {
			//if new segment intersects other segment
			QVector2D intPt;
			float tab, tcd;
			if (Util::segmentSegmentIntersectXY(p0, p1, roads.graph[*ei]->polyLine[i], roads.graph[*ei]->polyLine[i + 1], &tab, &tcd, true, intPt)) {
				float dist = (p0 - intPt).lengthSquared();

				//make sure we get only closest segment
				if (dist < min_dist) {
					min_dist = dist;
					eiClosest = *ei;
					closestIntPt = intPt;
					intersected = true;
				}
			}

		}
	}	

	return intersected;
}

/**
 * 近くの頂点にsnapすべきか、チェックする。
 * srcDescから伸ばしてきたエッジの先端posに近い頂点を探し、エッジの延長方向とのなす角度が90度以下で、距離が閾値以下のものがあるか探す。
 * 
 * @param pos				エッジ先端
 * @param threshold			距離の閾値
 * @param srcDesc			この頂点からエッジを延ばしている
 * @param snapDesc			最も近い頂点
 * @return					もしsnapすべき頂点があれば、trueを返却する
 */
bool RoadGeneratorHelper::canSnapToVertex(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc) {
	float min_dist = std::numeric_limits<float>::max();

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if (src == srcDesc || tgt == srcDesc) continue;

		if (QVector2D::dotProduct(roads.graph[src]->pt - roads.graph[srcDesc]->pt, pos - roads.graph[srcDesc]->pt) > 0) {
			float dist1 = (roads.graph[src]->pt - pos).lengthSquared();
			if (dist1 < min_dist) {
				min_dist = dist1;
				snapDesc = src;
			}
		}

		if (QVector2D::dotProduct(roads.graph[tgt]->pt - roads.graph[srcDesc]->pt, pos - roads.graph[srcDesc]->pt) > 0) {
			float dist2 = (roads.graph[tgt]->pt - pos).lengthSquared();
			if (dist2 < min_dist) {
				min_dist = dist2;
				snapDesc = tgt;
			}
		}
	}

	if (min_dist <= threshold * threshold) return true;
	else return false;
}

/**
 * 近くのエッジにsnapすべきか、チェックする。
 * srcDescから伸ばしてきたエッジの先端posに近いエッジを探し、エッジの延長方向とのなす角度が90度以下で、距離が閾値以下のものがあるか探す。
 * 
 * @param pos				エッジ先端
 * @param threshold			距離の閾値
 * @param srcDesc			この頂点からエッジを延ばしている
 * @param snapDesc			最も近い頂点
 * @return					もしsnapすべき頂点があれば、trueを返却する
 */
bool RoadGeneratorHelper::canSnapToEdge(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt) {
	float min_dist = std::numeric_limits<float>::max();

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// Highwayとは、intersectさせない
		if (roads.graph[*ei]->type == RoadEdge::TYPE_HIGHWAY) continue;

		QVector2D closePt;
		float dist = GraphUtil::distance(roads, pos, *ei, closePt);

		if (QVector2D::dotProduct(closePt - roads.graph[srcDesc]->pt, pos - roads.graph[srcDesc]->pt) > 0) {
			if (dist < min_dist) {
				min_dist = dist;
				snapEdge = *ei;
				closestPt = closePt;
			}
		}
	}		

	if (min_dist < threshold) return true;
	else return false;
}

/**
 * 指定された位置posに最も近い頂点snapDescを取得し、そこまでの距離を返却する。
 * ただし、srcDesc、又は、その隣接頂点は、スナップ対象外とする。
 * 現在、この関数は使用されていない。
 */
float RoadGeneratorHelper::getNearestVertex(RoadGraph& roads, const QVector2D& pos, RoadVertexDesc srcDesc, RoadVertexDesc& snapDesc) {
	float min_dist = std::numeric_limits<float>::max();

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if (src == srcDesc || tgt == srcDesc) continue;

		float dist1 = (roads.graph[src]->pt - pos).lengthSquared();
		float dist2 = (roads.graph[tgt]->pt - pos).lengthSquared();
		if (dist1 < min_dist) {
			min_dist = dist1;
			snapDesc = src;
		}
		if (dist2 < min_dist) {
			min_dist = dist2;
			snapDesc = tgt;
		}
	}

	return min_dist;
}

/**
 * 指定された位置posに最も近いエッジsnapEdgeを取得し、そこまでの距離を返却する。
 * ただし、srcDesc、又は、その隣接頂点は、スナップ対象外とする。
 * 現在、この関数は使用されていない。
 */
float RoadGeneratorHelper::getNearestEdge(RoadGraph& roads, const QVector2D& pt, RoadVertexDesc srcDesc, RoadEdgeDesc& snapEdge, QVector2D &closestPt) {
	float min_dist = std::numeric_limits<float>::max();
	RoadEdgeDesc min_e;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if (!roads.graph[src]->valid) continue;
		if (!roads.graph[tgt]->valid) continue;

		if (src == srcDesc || tgt == srcDesc) continue;

		QVector2D pt2;
		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			float dist = Util::pointSegmentDistanceXY(roads.graph[*ei]->polyLine[i], roads.graph[*ei]->polyLine[i + 1], pt, pt2);
			if (dist < min_dist) {
				min_dist = dist;
				snapEdge = *ei;
				closestPt = pt2;
			}
		}
	}

	return min_dist;
}

/**
 * カーネル設定済みの頂点の中から、直近のものを探す。
 * 現在、KDERoadGeneratorクラスでのみ使用。KDERoadGenerator2クラスでは使用していない。
 */
RoadVertexDesc RoadGeneratorHelper::getNearestVertexWithKernel(RoadGraph &roads, const QVector2D &pt) {
	RoadVertexDesc nearest_desc;
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		// カーネルのない頂点はスキップ
		if (roads.graph[*vi]->kernel.id == -1) continue;

		float dist = (roads.graph[*vi]->getPt() - pt).lengthSquared();
		if (dist < min_dist) {
			nearest_desc = *vi;
			min_dist = dist;
		}
	}

	return nearest_desc;
}

/**
 * 指定された点が、いずれかの頂点のテリトリーに入っているかチェックする。
 * ただし、頂点srcVertexは除く。
 * また、対象となる頂点へ伸びていて、且つ、対象となる頂点から、頂点srcVertexへもエッジが来る場合も、除外する。
 */
bool RoadGeneratorHelper::invadingTerritory(RoadGraph &roads, const QVector2D &pt, RoadVertexDesc srcVertex, const QVector2D &targetPt) {
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (*vi == srcVertex) continue;
		if (!roads.graph[*vi]->valid) continue;

		// ベクトルsrcVertex→*viと、ベクトルsrcVertex→ptが、逆方向なら、スキップ
		if (QVector2D::dotProduct(roads.graph[*vi]->pt - roads.graph[srcVertex]->pt, pt - roads.graph[srcVertex]->pt) < 0) continue;

		// カーネルのない頂点はスキップ
		if (roads.graph[*vi]->kernel.id == -1) continue;

		// 誤差により、テリトリーに入っていると判断されてしまうのを防ぐため、0.9fをかける。
		if ((roads.graph[*vi]->pt - pt).lengthSquared() < roads.graph[*vi]->kernel.territory * roads.graph[*vi]->kernel.territory * 0.81f) {
			// 対象となる頂点方向へ、エッジが伸びていない場合（角度が15度より大きい）は、「侵入」と判断する
			if (Util::diffAngle(roads.graph[*vi]->pt - pt, targetPt - pt) > M_PI * 15.0f / 180.0f) return true;

			if (roads.graph[*vi]->kernel.id == -1) return true;

			// 対象となる頂点から、頂点srcVertex方向へ向かうエッジがなければ、「侵入」と判断する
			bool close = false;
			for (int i = 0; i < roads.graph[*vi]->kernel.edges.size(); ++i) {
				if (Util::diffAngle(roads.graph[*vi]->kernel.edges[i].edge.last(), roads.graph[srcVertex]->pt - roads.graph[*vi]->pt) < M_PI * 15.0f / 180.0f) {
					close = true;
					break;
				}
			}

			if (!close) return true;

			continue;
		}
	}

	return false;
}

/**
 * カーネルの中で、指定された位置に最も近いものを探し、そのインデックスを返却する。
 */
int RoadGeneratorHelper::getClosestItem(const KDEFeature &f, int roadType, const QVector2D &pt) {
	float min_dist = std::numeric_limits<float>::max();
	int min_index = -1;
	for (int i = 0; i < f.items(roadType).size(); ++i) {
		float dist = (f.items(roadType)[i].pt - pt).lengthSquared();
		if (dist < min_dist) {
			min_dist = dist;
			min_index = i;
		}
	}

	return min_index;
}

/**
 * 指定された頂点について、指定されたエッジに似たエッジが既に登録済みかどうかチェックする。
 */
bool RoadGeneratorHelper::isRedundantEdge(RoadGraph& roads, RoadVertexDesc v_desc, const Polyline2D &polyline) {
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = out_edges(v_desc, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		if (roads.graph[*ei]->polyLine.size() <= 1) continue;
		if (polyline.size() <= 0) continue;

		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		Polyline2D edge = roads.graph[*ei]->polyLine;
		if ((roads.graph[v_desc]->pt - roads.graph[*ei]->polyLine[0]).lengthSquared() > (roads.graph[tgt]->pt - roads.graph[*ei]->polyLine[0]).lengthSquared()) {
			std::reverse(edge.begin(), edge.end());
		}

		for (int i = 1; i < 10; ++i) {
			int index1 = (edge.size() - 1) * i / 10;
			int index2 = polyline.size() * i/ 10;

			if (index1 == 0) continue;

			if (Util::diffAngle(edge[index1] - edge[0], polyline[index2]) < 0.3f) return true;
		}
	}

	return false;
}

/**
 * エリアに対して、指定された位置のモジュラ位置を返却する。
 */
QVector2D RoadGeneratorHelper::modulo(const Polygon2D &area, const QVector2D &pt) {
	QVector2D ret;

	BBox bbox = area.envelope();

	if (pt.x() < bbox.minPt.x()) {
		ret.setX(bbox.maxPt.x() - (int)(bbox.minPt.x() - pt.x()) % (int)bbox.dx());
	} else if (pt.x() > bbox.maxPt.x()) {
		ret.setX(bbox.minPt.x() + (int)(pt.x() - bbox.maxPt.x()) % (int)bbox.dx());
	} else {
		ret.setX(pt.x());
	}

	if (pt.y() < bbox.minPt.y()) {
		ret.setY(bbox.maxPt.y() - (int)(bbox.minPt.y() - pt.y()) % (int)bbox.dy());
	} else if (pt.y() > bbox.maxPt.y()) {
		ret.setY(bbox.minPt.y() + (int)(pt.y() - bbox.maxPt.y()) % (int)bbox.dy());
	} else {
		ret.setY(pt.y());
	}

	return ret;
}

void RoadGeneratorHelper::buildGraphFromKernel(RoadGraph& roads, const KDEFeatureItem &item, const QVector2D &offset) {
	roads.clear();

	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(item.pt + offset));
	RoadVertexDesc v_desc = GraphUtil::addVertex(roads, v);

	for (int i = 0; i < item.edges.size(); ++i) {
		RoadVertexPtr u = RoadVertexPtr(new RoadVertex(item.edges[i].edge.last()));
		RoadVertexDesc u_desc = GraphUtil::addVertex(roads, u);

		Polyline2D polyline;
		polyline.push_back(roads.graph[v_desc]->pt);
		for (int j = 0; j < item.edges[i].edge.size(); ++j) {
			polyline.push_back(roads.graph[v_desc]->pt + item.edges[i].edge[j]);
		}

		RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, v_desc, u_desc, 1, false);
		roads.graph[e_desc]->polyLine = polyline;
		roads.graph[e_desc]->color = QColor(192, 192, 255);
		roads.graph[e_desc]->bgColor = QColor(0, 0, 192);
	}
}
