﻿#include <limits>
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
		if ((roads.graph[*vi]->pt - pt).lengthSquared() < roads.graph[*vi]->kernel.territory * roads.graph[*vi]->kernel.territory * 0.9f) {
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
