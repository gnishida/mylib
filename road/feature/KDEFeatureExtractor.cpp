#include <time.h>
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
	time_t start = clock();
	GraphUtil::copyRoads(roads, temp_roads);
	time_t end = clock();
	std::cout << "Elapsed time for copying the roads: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::extractRoads(temp_roads, RoadEdge::TYPE_AVENUE | RoadEdge::TYPE_BOULEVARD);
	end = clock();
	std::cout << "Elapsed time for extracting only the avenues: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::clean(temp_roads);
	end = clock();
	std::cout << "Elapsed time for cleaning the avenues: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::reduce(temp_roads);
	end = clock();
	std::cout << "Elapsed time for reducing the avenues: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;

	// roundaboutを削除する
	//GraphUtil::removeRoundabout(temp_roads);

	// linkを削除する
	start = clock();
	GraphUtil::removeLinkEdges(temp_roads);
	end = clock();
	std::cout << "Elapsed time for removing links: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::reduce(temp_roads);
	end = clock();
	std::cout << "Elapsed time for reducing links: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::clean(temp_roads);
	end = clock();
	std::cout << "Elapsed time for cleaning the avenues: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;

	start = clock();
	int num_vertices = 0;
	int id = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(temp_roads.graph); vi != vend; ++vi) {
		if (!temp_roads.graph[*vi]->valid) continue;

		// エリア外の頂点はスキップ
		if (!area.contains(temp_roads.graph[*vi]->pt)) continue;

		num_vertices++;

		// エッジの数が2以下なら、スキップ
		if (GraphUtil::getNumEdges(temp_roads, *vi) <= 2) continue;

		// 頂点の座標の、エリア中心からのオフセットを登録
		KDEFeatureItem item(id);
		item.pt = temp_roads.graph[*vi]->pt - center;

		// 近接頂点までの距離を登録
		RoadVertexDesc nearestVertexDesc = GraphUtil::getVertex(temp_roads, temp_roads.graph[*vi]->pt, *vi);
		item.territory = (temp_roads.graph[nearestVertexDesc]->pt - temp_roads.graph[*vi]->pt).length();

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
		id++;
	}
	end = clock();
	std::cout << "Elapsed time for extracting features from the avenues: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;

	BBox bbox = area.envelope();
	std::cout << "Area: " << area.area() << std::endl;
	std::cout << "BBox: " << bbox.dx() << "," << bbox.dy() << std::endl;
	std::cout << "Num avenue vertices: " << num_vertices << std::endl;

	kf->setDensity(RoadEdge::TYPE_AVENUE, num_vertices / area.area());

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	// streetのみを抽出する
	start = clock();
	GraphUtil::copyRoads(roads, temp_roads);
	end = clock();
	std::cout << "Elapsed time for copying the roads: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::extractRoads(temp_roads, RoadEdge::TYPE_STREET);
	end = clock();
	std::cout << "Elapsed time for extracting the streets: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	//GraphUtil::reduce(temp_roads);  <- わざとreduceしない

	// linkを削除する
	start = clock();
	GraphUtil::removeLinkEdges(temp_roads);
	end = clock();
	std::cout << "Elapsed time for removing links: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;
	start = clock();
	GraphUtil::clean(temp_roads);
	end = clock();
	std::cout << "Elapsed time for cleaning the streets: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;

	start = clock();
	num_vertices = 0;
	id = 0;
	for (boost::tie(vi, vend) = boost::vertices(temp_roads.graph); vi != vend; ++vi) {
		if (!temp_roads.graph[*vi]->valid) continue;

		// エリア外の頂点はスキップ
		if (!area.contains(temp_roads.graph[*vi]->pt)) continue;

		num_vertices++;

		// エッジの数が2以下なら、スキップ
		//if (GraphUtil::getNumEdges(temp_roads, *vi) <= 2) continue;

		// 頂点の座標の、エリア中心からのオフセットを登録
		KDEFeatureItem item(id);
		item.pt = temp_roads.graph[*vi]->pt - center;

		// 近接頂点までの距離を登録
		RoadVertexDesc nearestVertexDesc = GraphUtil::getVertex(temp_roads, temp_roads.graph[*vi]->pt, *vi);
		item.territory = (temp_roads.graph[nearestVertexDesc]->pt - temp_roads.graph[*vi]->pt).length();

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
		id++;
	}
	end = clock();
	std::cout << "Elapsed time for extracting features from the streets: " << (double)(end-start)/CLOCKS_PER_SEC << " [sec]" << std::endl;

	kf->setDensity(RoadEdge::TYPE_STREET, num_vertices / area.area());

	kf->setWeight(1.0f);
	kf->setCenter(area.centroid());
	kf->setArea(area);

	roadFeature.addFeature(kf);
}
