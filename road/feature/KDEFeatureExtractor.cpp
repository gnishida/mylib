#include "../GraphUtil.h"
#include "KDEFeatureExtractor.h"

/**
 * KDEベースでの特徴量を抽出する。
 */
void KDEFeatureExtractor::extractFeature(RoadGraph& roads, Polygon2D& area, RoadFeature& roadFeature) {
	roadFeature.clear();

	KDEFeaturePtr kf = KDEFeaturePtr(new KDEFeature(0));

	QVector2D center = area.centroid();

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	// Avenueのみを抽出する
	RoadGraph temp_roads;
	GraphUtil::copyRoads(roads, temp_roads);
	GraphUtil::extractRoads(temp_roads, RoadEdge::TYPE_AVENUE);
	GraphUtil::clean(temp_roads);
	GraphUtil::reduce(temp_roads);

	// roundaboutを削除する
	//GraphUtil::removeRoundabout(temp_roads);

	// linkを削除する
	GraphUtil::removeLinkEdges(temp_roads);
	GraphUtil::reduce(temp_roads);
	GraphUtil::clean(temp_roads);

	int num_vertices = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(temp_roads.graph); vi != vend; ++vi) {
		if (!temp_roads.graph[*vi]->valid) continue;

		// エリア外の頂点はスキップ
		if (!area.contains(temp_roads.graph[*vi]->pt)) continue;

		num_vertices++;

		// エッジの数が2以下なら、スキップ
		if (GraphUtil::getNumEdges(temp_roads, *vi) <= 2) continue;

		// 頂点の座標の、エリア中心からのオフセットを登録
		KDEFeatureItem item;
		item.pt = temp_roads.graph[*vi]->pt - center;

		// 各outing edgeを登録
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, temp_roads.graph); ei != eend; ++ei) {
			RoadVertexDesc tgt = boost::target(*ei, temp_roads.graph);
			int degree = GraphUtil::getNumEdges(temp_roads, tgt);

			Polyline2D polyline = GraphUtil::finerEdge(temp_roads, *ei, 20.0f);
			if ((polyline[0] - temp_roads.graph[*vi]->pt).lengthSquared() > (polyline[0] - temp_roads.graph[tgt]->pt).lengthSquared()) {
				std::reverse(polyline.begin(), polyline.end());
			}
			for (int i = 1; i < polyline.size(); ++i) {
				polyline[i] -= polyline[0];
			}
			polyline.erase(polyline.begin());
			item.addEdge(polyline, degree == 1);
		}

		kf->addItem(RoadEdge::TYPE_AVENUE, item);
	}

	BBox bbox = area.envelope();
	std::cout << "Area: " << area.area() << std::endl;
	std::cout << "BBox: " << bbox.dx() << "," << bbox.dy() << std::endl;
	std::cout << "Num avenue vertices: " << num_vertices << std::endl;

	kf->setDensity(RoadEdge::TYPE_AVENUE, num_vertices / area.area());

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	// streetのみを抽出する
	GraphUtil::copyRoads(roads, temp_roads);
	GraphUtil::extractRoads(temp_roads, RoadEdge::TYPE_STREET);
	GraphUtil::clean(temp_roads);
	//GraphUtil::reduce(temp_roads);  <- わざとreduceしない

	// linkを削除する
	GraphUtil::removeLinkEdges(temp_roads);
	GraphUtil::reduce(temp_roads);
	GraphUtil::clean(temp_roads);

	num_vertices = 0;
	for (boost::tie(vi, vend) = boost::vertices(temp_roads.graph); vi != vend; ++vi) {
		if (!temp_roads.graph[*vi]->valid) continue;

		// エリア外の頂点はスキップ
		if (!area.contains(temp_roads.graph[*vi]->pt)) continue;

		num_vertices++;

		// エッジの数が2以下なら、スキップ
		if (GraphUtil::getNumEdges(temp_roads, *vi) <= 2) continue;

		// 頂点の座標の、エリア中心からのオフセットを登録
		KDEFeatureItem item;
		item.pt = temp_roads.graph[*vi]->pt - center;

		// 各outing edgeを登録
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, temp_roads.graph); ei != eend; ++ei) {
			RoadVertexDesc tgt = boost::target(*ei, temp_roads.graph);
			int degree = GraphUtil::getNumEdges(temp_roads, tgt);

			Polyline2D polyline;
			if ((temp_roads.graph[*ei]->polyLine[0] - temp_roads.graph[*vi]->pt).lengthSquared() > (temp_roads.graph[*ei]->polyLine[0] - temp_roads.graph[tgt]->pt).lengthSquared()) {
				std::reverse(temp_roads.graph[*ei]->polyLine.begin(), temp_roads.graph[*ei]->polyLine.end());
			}
			for (int i = 1; i < temp_roads.graph[*ei]->polyLine.size(); ++i) {
				polyline.push_back(temp_roads.graph[*ei]->polyLine[i] - temp_roads.graph[*vi]->pt);
			}
			item.addEdge(polyline, degree == 1);
		}

		kf->addItem(RoadEdge::TYPE_STREET, item);
	}

	kf->setDensity(RoadEdge::TYPE_STREET, num_vertices / area.area());

	kf->setWeight(1.0f);
	kf->setCenter(area.centroid());
	kf->setArea(area);

	roadFeature.addFeature(kf);
}
