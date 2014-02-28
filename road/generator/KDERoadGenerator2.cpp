#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include "../../common/Util.h"
#include "../../common/TopNSearch.h"
#include "../GraphUtil.h"
#include "KDERoadGenerator2.h"
#include "RoadGeneratorHelper.h"

void KDERoadGenerator2::generateRoadNetwork(RoadGraph &roads, const Polygon2D &area, const KDEFeature& kf, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, bool addAvenuesOnBoundary, int numIterations, bool isGenerateLocalStreets) {
	srand(12345);

	std::list<RoadVertexDesc> seeds;

	while (true) {
		// Avenueのシードを生成
		generateAvenueSeeds(roads, area, kf, seeds);

		if (seeds.empty()) break;

		// Avenueを生成
		for (int i = 0; !seeds.empty() && i < numIterations; ++i) {
			RoadVertexDesc desc = seeds.front();
			seeds.pop_front();

			std::cout << "attemptExpansion (avenue): " << i << " (Seed: " << desc << ")" << std::endl;
			//if (i == 17) break;
			attemptExpansion(roads, area, desc, RoadEdge::TYPE_AVENUE, kf, invadingCheck, weightEdge, weightLocation, weightRepetition, seeds);
		}

		// clean up
		GraphUtil::clean(roads);
		//GraphUtil::reduce(roads);
		//GraphUtil::clean(roads);

		break;
	}

	// 境界上に、Avenueを生成
	if (addAvenuesOnBoundary) {
		generateRoadsOnBoundary(roads, area, RoadEdge::TYPE_AVENUE, 1);
	}

	if (!isGenerateLocalStreets) {
		GraphUtil::clean(roads);
		return;
	}

	// Avenueをできる限りつなぐ
	//connectAvenues(roads, 400.0f);

	// Local streetを生成
	generateStreetSeeds(roads, area, kf, seeds);

	for (int i = 0; !seeds.empty() && i < numIterations; ++i) {
		RoadVertexDesc desc = seeds.front();
		seeds.pop_front();

		std::cout << "attemptExpansion (street): " << i << std::endl;
		//if (i == 3) break;
		attemptExpansion(roads, area, desc, RoadEdge::TYPE_STREET, kf, invadingCheck, weightEdge, weightLocation, weightRepetition, seeds);
	}

	// isolated edgeを削除
	//GraphUtil::removeIsolatedEdges(roads);
	//GraphUtil::clean(roads);
}

/**
 * Areaの境界上に、道路を生成する。
 */
void KDERoadGenerator2::generateRoadsOnBoundary(RoadGraph &roads, const Polygon2D &area, int roadType, int lanes) {
	RoadVertexDesc prevDesc;

	for (int i = 0; i < area.size(); ++i) {
		RoadVertexDesc desc;
		if (!GraphUtil::getVertex(roads, area[i], 0.1f, desc)) {
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(area[i]));
			desc = GraphUtil::addVertex(roads, v);
		}

		if (i > 0) {
			GraphUtil::addEdge(roads, prevDesc, desc, roadType, lanes);
		}

		prevDesc = desc;
	}
}

/**
 * シード頂点を生成する。
 * 密度に応じて、エリア内にランダムにシードを生成する。
 */
void KDERoadGenerator2::generateAvenueSeeds(RoadGraph &roads, const Polygon2D &area, const KDEFeature& f, std::list<RoadVertexDesc>& seeds) {
	seeds.clear();

	QVector2D center = area.centroid();
	
	float numExpectedVertices = f.density(RoadEdge::TYPE_AVENUE) * area.area();
	int numSeeds = numExpectedVertices / 30 + 1;
	float threshold = area.area() / numExpectedVertices;

	std::cout << "Area: " << area.area() << std::endl;
	std::cout << "Expected num vertices: " << numExpectedVertices << std::endl;
	std::cout << "Num seeds: " << numSeeds << std::endl;

	QSet<int> usedKernels;
	addAvenueSeed(roads, area, f, QVector2D(0, 0), usedKernels, seeds);

	// onBoundary頂点について、このエリア内であれば、シードとして使用する
	/*
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		if (roads.graph[*vi]->onBoundary && area.contains(roads.graph[*vi]->pt)) {
			RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads.graph[*vi]->pt));
			RoadVertexDesc new_v_desc = GraphUtil::addVertex(roads, new_v);

			seeds.push_back(new_v_desc);
		}
	}
	*/
}

void KDERoadGenerator2::addAvenueSeed(RoadGraph &roads, const Polygon2D &area, const KDEFeature &f, const QVector2D &offset, QSet<int> &usedKernels, std::list<RoadVertexDesc>& seeds) {
	QVector2D center = area.centroid();

	// Avenueカーネルの中で、offsetの位置に最も近いものを探す
	int min_index = RoadGeneratorHelper::getClosestItem(f, RoadEdge::TYPE_AVENUE, offset);

	// もし、シードとして使用済みのカーネルなら、シードとして追加せずにキャンセル
	if (usedKernels.contains(min_index)) return;

	usedKernels.insert(min_index);

	// 頂点を追加し、シードとする
	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(f.items(RoadEdge::TYPE_AVENUE)[min_index].pt + center));
	v->kernel = f.items(RoadEdge::TYPE_AVENUE)[min_index];
	RoadVertexDesc desc = GraphUtil::addVertex(roads, v);
	seeds.push_back(desc);
}

/**
 * Local street用のシードを生成する。
 * Avenueが既に生成済みであることを前提とする。Avenueにより生成されるFaceを抽出し、その中心をシードとする。
 */
void KDERoadGenerator2::generateStreetSeeds(RoadGraph &roads, const Polygon2D &area, const KDEFeature& f, std::list<RoadVertexDesc>& seeds) {
}

void KDERoadGenerator2::attemptExpansion(RoadGraph &roads, const Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, std::list<RoadVertexDesc> &seeds) {
	QVector2D center = area.centroid();

	KDEFeatureItem item = roads.graph[srcDesc]->kernel;
	if (item.id == -1) {
		item = getItem(roads, area, f, roadType, srcDesc, weightEdge, weightLocation, weightRepetition);
		//KDEFeatureItem item = getItem2(roads, area, f, roadType, srcDesc);//, roads.graph[srcDesc]->pt - center);
		roads.graph[srcDesc]->kernel = item;
	}
	
	for (int i = 0; i < item.edges.size(); ++i) {
		growRoadSegment(roads, area, srcDesc, roadType, f, item.edges[i], invadingCheck, weightEdge, weightLocation, weightRepetition, seeds);
	}
}

/**
 * 指定されたpolylineに従って、srcDesc頂点からエッジを伸ばす。
 * シードとして追加された場合は、trueを返却する
 */
bool KDERoadGenerator2::growRoadSegment(RoadGraph &roads, const Polygon2D &area, RoadVertexDesc &srcDesc, int roadType, const KDEFeature& f, const KDEFeatureItemEdge &edge, bool invadingCheck, float weightEdge, float weightLocation, float weightRepetition, std::list<RoadVertexDesc> &seeds) {
	RoadVertexDesc tgtDesc;
	RoadVertexDesc snapDesc;

	bool snapped = false;
	bool intersected = false;
	bool outside = false;
	QVector2D outsidePt;

	bool toBeSeed = true;

	Polyline2D polyline;
	polyline.push_back(roads.graph[srcDesc]->pt);

	QVector2D pt;
	for (int j = 0; j < edge.edge.size(); ++j) {
		pt = roads.graph[srcDesc]->pt + edge.edge[j];

		// INTERSECTS -- If edge intersects other edge
		QVector2D intPoint;
		RoadEdgeDesc closestEdge;
		intersected = RoadGeneratorHelper::intersects(roads, roads.graph[srcDesc]->pt, pt, closestEdge, intPoint);
		if (intersected) {
			RoadVertexDesc src = boost::source(closestEdge, roads.graph);
			RoadVertexDesc tgt = boost::target(closestEdge, roads.graph);

			// 自分のエッジに交差した場合は、このエッジのgrowをキャンセル
			if (src == srcDesc || tgt == srcDesc) return false;

			pt = intPoint;
		}

		// Densityをチェック
		if (roadType == RoadEdge::TYPE_STREET) {
			float density = GraphUtil::getNumVertices(roads, pt, 400);
			//if (density >= (f.density(roadType) + f.density(RoadEdge::TYPE_AVENUE)) * 400.0f * 400.0f * M_PI * 3) return false;
		} else {
			//float density = GraphUtil::getNumVertices(roads, pt, 400);
			//if (density >= f.density(roadType) * 400.0f * 400.0f * M_PI) return false;
		}

		float threshold;
		if (roadType == RoadEdge::TYPE_STREET || j < edge.edge.size() - 1) {
			//threshold = std::max(0.25f * (float)edge[j].length(), 10.0f);
			threshold = std::min(0.25f * (float)edge.edge[j].length(), 10.0f);
		} else {
			//threshold = (std::max)(0.25f * (float)edge[j].length(), 40.0f);
			threshold = std::min(0.5f * (float)edge.edge[j].length(), 40.0f);
		}

		// 近くに頂点があるか？
		RoadVertexDesc desc;
		RoadEdgeDesc e_desc;
		QVector2D closestPt;		
		if (RoadGeneratorHelper::canSnapToVertex(roads, pt, threshold, srcDesc, desc)) {
			snapDesc = desc;
			snapped = true;
			intersected = false;
			toBeSeed = false;
		} else if (RoadGeneratorHelper::canSnapToEdge(roads, pt, threshold, srcDesc, e_desc, closestPt)) {
			// 実験。既存のエッジを分割させないよう、キャンセルさせてみる
			if (roadType == RoadEdge::TYPE_AVENUE && roads.graph[e_desc]->type == RoadEdge::TYPE_AVENUE) {
				//return false;
			} 
			
			toBeSeed = false;

			snapDesc = GraphUtil::splitEdge(roads, e_desc, pt);
			snapped = true;
			intersected = false;
		} else {
			if (!outside && !area.contains(pt)) {
				// エリア外周との交点を求める
				area.intersects(roads.graph[srcDesc]->pt, pt, outsidePt);
				pt = outsidePt;
				outside = true;
				toBeSeed = false;
			}
		}

		if (intersected) {
			// 交差相手のエッジを分割
			tgtDesc = GraphUtil::splitEdge(roads, closestEdge, pt);
		}
			
		polyline.push_back(pt);

		if (snapped || intersected || outside) break;
	}

	if (!intersected) {
		// 他の頂点のテリトリーに侵入したら、頂点生成、エッジ生成を却下する
		if (RoadGeneratorHelper::invadingTerritory(roads, pt, srcDesc, roads.graph[srcDesc]->pt + edge.edge[edge.edge.size() - 1])) return false;

		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
		tgtDesc = GraphUtil::addVertex(roads, v);
			
		if (outside) {
			roads.graph[tgtDesc]->onBoundary = true;
		}
	}

	if (GraphUtil::hasEdge(roads, srcDesc, tgtDesc)) return false;

	// エッジを追加
	RoadEdgeDesc e = GraphUtil::addEdge(roads, srcDesc, tgtDesc, roadType, 1);
	roads.graph[e]->polyLine = polyline;


	if (snapped) {
		//GraphUtil::addEdge(roads, tgtDesc, snapDesc, roadType, 1);
		GraphUtil::snapVertex(roads, tgtDesc, snapDesc);
	}

	// シードに追加
	//if (!snapped && !intersected && !outside && !edge.deadend) {
	if (toBeSeed) {
		seeds.push_back(tgtDesc);

		// 追加した頂点に、カーネルを割り当てる
		//QVector2D offsetPos = roads.graph[srcDesc]->kernel.pt + roads.graph[tgtDesc]->pt - roads.graph[srcDesc]->pt;
		//std::reverse(polyline.begin(), polyline.end());
		roads.graph[tgtDesc]->kernel = getItem(roads, area, f, roadType, tgtDesc, weightEdge, weightLocation, weightRepetition);
	}

	return true;
}

/**
 * 与えられたエッジの方向、長さを含むデータを検索し、近いものを返却する。
 *
 * @param kf					特徴量
 * @param roadType				道路タイプ
 * @param offsetPosOfVertex		与えられた頂点の、このエリアの中心からのオフセット位置
 */
KDEFeatureItem KDERoadGenerator2::getItem(RoadGraph &roads, const Polygon2D &area, const KDEFeature& kf, int roadType, RoadVertexDesc v_desc, float weightEdge, float weightLocation, float weightRepetition) {
	// 与えられた頂点の、このエリアの中心からのオフセット位置を計算
	QVector2D offsetPosOfVertex = roads.graph[v_desc]->pt - area.centroid();

	// 当該頂点から出るエッジをリストアップする
	QList<Polyline2D> polylines;
	QList<RoadVertexDesc> neighbors;
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v_desc, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		neighbors.push_back(tgt);

		if ((roads.graph[v_desc]->pt - roads.graph[*ei]->polyLine[0]).lengthSquared() > (roads.graph[tgt]->pt - roads.graph[*ei]->polyLine[0]).lengthSquared()) {
			std::reverse(roads.graph[*ei]->polyLine.begin(), roads.graph[*ei]->polyLine.end());
		}
		polylines.push_back(roads.graph[*ei]->polyLine);
	}

	// 周辺の頂点のカーネルをリストアップし、最短距離を計算する
	/*float threshold = 200.0f;
	if (roadType == RoadEdge::TYPE_STREET) {
		threshold = 10.0f;
	}
	float threshold2 = threshold * threshold;*/
	QMap<int, float> neighborKernels;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		// 自分自身ならスキップ
		if (*vi == v_desc) continue;

		//if ((roads.graph[*vi]->pt - roads.graph[v_desc]->pt).lengthSquared() <= threshold2) {
			float min_dist = std::numeric_limits<float>::max();
			if (neighborKernels.contains(roads.graph[*vi]->kernel.id)) {
				min_dist = neighborKernels[roads.graph[*vi]->kernel.id];
			}
			float dist = (roads.graph[*vi]->pt - roads.graph[v_desc]->pt).length();
			if (dist < min_dist) {
				neighborKernels[roads.graph[*vi]->kernel.id] = dist;
			}
		//}
	}

	// 各カーネルについて、非類似度スコアを計算する
	//float edge_weight = 1.0f;
	//float location_weight = 1.0f;
	//float repetition_weight = 2000000.0f;

	float min_diff = std::numeric_limits<float>::max();
	int min_index = -1;
	for (int i = 0; i < kf.items(roadType).size(); ++i) {
		// 繰り返し度を計算
		float repetition = 0.0f;
		if (neighborKernels.contains(kf.items(roadType)[i].id)) {
			repetition = 1.0f / neighborKernels[kf.items(roadType)[i].id];
		}

		// エッジの非類似度を計算
		float edge_diff = 0.0f;
		for (int j = 0; j < polylines.size(); ++j) {
			edge_diff += kf.items(roadType)[i].getMinDistance(polylines[j]);
		}		

		// 位置の非類似度を計算
		float location_diff = 0.0f;
		//if (kf.area().contains(offsetPosOfVertex)) { // 位置が元のエリア内なら
		//	location_diff += (kf.items(roadType)[i].pt - offsetPosOfVertex).length();
		//} else {
			for (int j = 0; j < neighbors.size(); ++j) { // 位置が元のエリア外なら、隣接頂点のカーネルの共起性を考慮
				location_diff += (roads.graph[neighbors[j]]->kernel.pt - polylines[j].last() + polylines[j][0] - kf.items(roadType)[i].pt).length();
			}
		//}

		// フィッティングスコアを計算
		float diff;
		if (kf.area().contains(offsetPosOfVertex)) { // 位置が元のエリア内なら
			diff = location_diff;
		} else {
			diff = edge_diff * weightEdge + location_diff * weightLocation + repetition * weightRepetition;
		}
		if (diff < min_diff) {
			min_diff = diff;
			min_index = i;
		}
	}

	KDEFeatureItem item = kf.items(roadType)[min_index];

	// 与えられたエッジの方向に近いエッジを削除する
	for (int j = 0; j < polylines.size(); ++j) {
		float min_angle = std::numeric_limits<float>::max();
		int min_edge_index = -1;
		for (int i = 0; i < item.edges.size(); ++i) {
			float angle = Util::diffAngle(item.edges[i].edge[0], polylines[j][1] - polylines[j][0]);
			if (angle < min_angle) {
				min_angle = angle;
				min_edge_index = i;
			}
		}
		if (min_angle < 0.6f) {
			item.edges.erase(item.edges.begin() + min_edge_index);
		}
	}

	return item;
}

/**
 * degree=1の頂点について、近くの頂点とできる限りつなぐ。
 */
void KDERoadGenerator2::connectAvenues(RoadGraph &roads, float threshold) {
	float threshold2 = threshold * threshold;

	std::vector<RoadVertexDesc> deadendVertices;

	// degree = 1の頂点リストを取得
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;
		
		// 境界の頂点は、除外する
		if (roads.graph[*vi]->onBoundary) continue;

		if (GraphUtil::getDegree(roads, *vi) == 1) {
			deadendVertices.push_back(*vi);
		}
	}

	// degree=1の各頂点について、最も近い頂点またはエッジを探す
	for (int i = 0; i < deadendVertices.size(); ++i) {
		// 最も近い頂点を探す
		float min_dist_v = std::numeric_limits<float>::max();
		RoadVertexDesc min_v_desc;
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			if (!roads.graph[*vi]->valid) continue;
			if (*vi == deadendVertices[i]) continue;

			float dist = (roads.graph[*vi]->pt - roads.graph[deadendVertices[i]]->pt).lengthSquared();
			if (dist < min_dist_v) {
				min_dist_v = dist;
				min_v_desc = *vi;
			}
		}

		// 最も近いエッジを探す
		QVector2D closestPt;
		float min_dist_e = std::numeric_limits<float>::max();
		RoadEdgeDesc min_e_desc;
		RoadEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			RoadVertexDesc src = boost::source(*ei, roads.graph);
			RoadVertexDesc tgt = boost::target(*ei, roads.graph);
			if (src == deadendVertices[i] || tgt == deadendVertices[i]) continue;

			QVector2D closePt;
			float dist = GraphUtil::distance(roads, roads.graph[deadendVertices[i]]->pt, *ei, closePt);
			if (dist < min_dist_e) {
				min_dist_e = dist;
				min_e_desc = *ei;
				closestPt = closePt;
			}
		}

		if (min_dist_v < threshold2) {
			//GraphUtil::snapVertex(roads, deadendVertices[i], min_v_desc);
			GraphUtil::addEdge(roads, deadendVertices[i], min_v_desc, RoadEdge::TYPE_AVENUE, 1);
		} else {
			RoadVertexDesc desc = GraphUtil::splitEdge(roads, min_e_desc, roads.graph[deadendVertices[i]]->pt);

			//GraphUtil::snapVertex(roads, deadendVertices[i], desc);
			GraphUtil::addEdge(roads, deadendVertices[i], desc, RoadEdge::TYPE_AVENUE, 1);
		}
	}
}

/**
 * 境界上の頂点を延長し、近くのエッジにぶつける
 */
void KDERoadGenerator2::connectRoads(RoadGraph &roads, RoadAreaSet &areas, float dist_threshold, float angle_threshold) {
	// 境界上の頂点、エッジの組をリストアップする
	QList<RoadVertexDesc> boundaryNodes;
	QMap<RoadVertexDesc, RoadEdgeDesc> boundaryEdges;
	QMap<RoadVertexDesc, RoadVertexDesc> boundaryNodesPair;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		
		if (roads.graph[src]->onBoundary && GraphUtil::getDegree(roads, src) == 1) {
			if (!boundaryNodes.contains(src)) boundaryNodes.push_back(src);
			if (!boundaryEdges.contains(src)) boundaryEdges[src] = *ei;
			if (!boundaryNodesPair.contains(src)) boundaryNodesPair[src] = tgt;
		} else if (roads.graph[tgt]->onBoundary && GraphUtil::getDegree(roads, tgt) == 1) {
			if (!boundaryNodes.contains(tgt)) boundaryNodes.push_back(tgt);
			if (!boundaryEdges.contains(tgt)) boundaryEdges[tgt] = *ei;
			if (!boundaryNodesPair.contains(tgt)) boundaryNodesPair[tgt] = src;
		}
	}

	// リストアップしたエッジを、それぞれ少しずつ伸ばしていき、他のエッジにぶつかったらストップする
	int numIterations = 1000;
	while (!boundaryNodes.empty() && numIterations >= 0) {
		RoadVertexDesc v_desc = boundaryNodes.front();
		boundaryNodes.pop_front();

		if (!roads.graph[v_desc]->valid) continue;

		RoadVertexDesc v2_desc = boundaryNodesPair[v_desc];
		RoadEdgeDesc e_desc = boundaryEdges[v_desc];

		QVector2D step;
		if ((roads.graph[v_desc]->pt - roads.graph[e_desc]->polyLine[0]).lengthSquared() <= (roads.graph[v2_desc]->pt - roads.graph[e_desc]->polyLine[0]).lengthSquared()) {
			step = roads.graph[e_desc]->polyLine[0] - roads.graph[e_desc]->polyLine[1];
		} else {
			step = roads.graph[e_desc]->polyLine.last() - roads.graph[e_desc]->polyLine[roads.graph[e_desc]->polyLine.size() - 2];
		}
		step = step.normalized() * 20.0f;

		if (growRoadOneStep(roads, v_desc, step)) {
			boundaryNodes.push_back(v_desc);
		}

		numIterations--;
	}

	GraphUtil::clean(roads);
}

bool KDERoadGenerator2::growRoadOneStep(RoadGraph& roads, RoadVertexDesc srcDesc, const QVector2D& step) {
	bool snapped = false;
	bool intersected = false;

	QVector2D pt = roads.graph[srcDesc]->pt + step;
	RoadEdgeDesc closestEdge;

	// INTERSECTS -- If edge intersects other edge
	QVector2D intPoint;
	intersected = RoadGeneratorHelper::intersects(roads, roads.graph[srcDesc]->pt, pt, closestEdge, intPoint);
	if (intersected) {
		pt = intPoint;
	}

	if (intersected) {
		RoadVertexDesc splitVertex = GraphUtil::splitEdge(roads, closestEdge, pt);
		GraphUtil::snapVertex(roads, srcDesc, splitVertex);

		// 交差相手のエッジが、成長中のエッジなら、その成長をストップする
		RoadVertexDesc src = boost::source(closestEdge, roads.graph);
		RoadVertexDesc tgt = boost::target(closestEdge, roads.graph);
		if (roads.graph[src]->onBoundary) {
			RoadEdgeDesc e = GraphUtil::getEdge(roads, src, splitVertex);
			roads.graph[e]->valid = false;
			roads.graph[src]->valid = false;
		} else if (roads.graph[tgt]->onBoundary) {
			RoadEdgeDesc e = GraphUtil::getEdge(roads, tgt, splitVertex);
			roads.graph[e]->valid = false;
			roads.graph[tgt]->valid = false;
		}

		return false;
	} else {
		GraphUtil::moveVertex(roads, srcDesc, pt);
		return true;
	}	
}

/**
 * 境界上の頂点を、できるだけ近くの他の頂点とつなぐ
 */
void KDERoadGenerator2::connectRoads2(RoadAreaSet &areas, float dist_threshold, float angle_threshold) {
	for (int i = 0; i < areas.size(); ++i) {
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = vertices(areas.areas[i].roads.graph); vi != vend; ++vi) {
			if (!areas.areas[i].roads.graph[*vi]->valid) continue;

			if (!areas.areas[i].roads.graph[*vi]->onBoundary) continue;

			connectRoads2(areas, i, *vi, dist_threshold, angle_threshold);
		}
	}

	// 繋がらなかった頂点については、その頂点と、そこから出るエッジを無効にする
	/*
	for (boost::tie(vi, vend) = vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		if (!roads.graph[*vi]->onBoundary) continue;

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = out_edges(*vi, roads.graph); ei != eend; ++ei) {
			roads.graph[*ei]->valid = false;
		}

		roads.graph[*vi]->valid = false;
	}
	*/

	//GraphUtil::clean(roads);
}

void KDERoadGenerator2::connectRoads2(RoadAreaSet &areas, int area_id, RoadVertexDesc v_desc, float dist_threshold, float angle_threshold) {
	float dist_threshold2 = dist_threshold * dist_threshold;

	float min_dist = std::numeric_limits<float>::max();
	int min_area_id;
	RoadVertexDesc min_desc;

	// v_descの、もう一端の頂点を取得
	RoadVertexDesc v2_desc;
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = out_edges(v_desc, areas.areas[area_id].roads.graph); ei != eend; ++ei) {
		if (!areas.areas[area_id].roads.graph[*ei]->valid) continue;

		v2_desc = boost::target(*ei, areas.areas[area_id].roads.graph);
	}

	for (int i = 0; i < areas.size(); ++i) {
		if (i == area_id) continue;

		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = vertices(areas.areas[i].roads.graph); vi != vend; ++vi) {
			if (!areas.areas[i].roads.graph[*vi]->valid) continue;

			//if (!roads.graph[*vi]->onBoundary) continue;

			/*
			for (boost::tie(ei, eend) = out_edges(*vi, roads.graph); ei != eend; ++ei) {
				if (!roads.graph[*ei]->valid) continue;

				RoadVertexDesc tgt = boost::target(*ei, roads.graph);
				float a = Util::diffAngle(roads.graph[*vi]->pt - roads.graph[tgt]->pt, roads.graph
			}
			*/

			float dist = (areas.areas[i].roads.graph[*vi]->pt - areas.areas[area_id].roads.graph[v_desc]->pt).lengthSquared();
			if (dist < min_dist) {
				min_dist = dist;
				min_area_id = i;
				min_desc = *vi;
			}
		}
	}

	//if (min_dist > dist_threshold2) return;

	// スナップにより、角度が大幅に変わる場合は、スナップさせない
	float angle = Util::diffAngle(areas.areas[area_id].roads.graph[v_desc]->pt - areas.areas[area_id].roads.graph[v2_desc]->pt, areas.areas[min_area_id].roads.graph[min_desc]->pt - areas.areas[area_id].roads.graph[v2_desc]->pt);
	if (angle > angle_threshold) return;

	GraphUtil::moveVertex(areas.areas[area_id].roads, v_desc, areas.areas[min_area_id].roads.graph[min_desc]->pt);
	areas.areas[area_id].roads.graph[v_desc]->onBoundary = false;
	areas.areas[min_area_id].roads.graph[min_desc]->onBoundary = false;
}
