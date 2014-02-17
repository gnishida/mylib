#include <QGLWidget>
#include "Util.h"
#include "GraphUtil.h"
#include "RoadGraph.h"

RoadGraph::RoadGraph() {
	modified = true;

	showHighways = true;
	showBoulevard = true;
	showAvenues = true;
	showLocalStreets = true;
}

RoadGraph::~RoadGraph() {
}

void RoadGraph::generateMesh() {
	if (!modified) return;

	renderables.clear();

	renderables.push_back(RenderablePtr(new Renderable(GL_TRIANGLES)));
	renderables.push_back(RenderablePtr(new Renderable(GL_POINTS, 10.0f)));

	// road edge
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(graph); ei != eend; ++ei) {
		if (!graph[*ei]->valid) continue;

		RoadEdgePtr edge = graph[*ei];

		QColor color, bgColor;
		float height;
		switch (edge->type) {
		case RoadEdge::TYPE_HIGHWAY:
			height = highwayHeight;
			break;
		case RoadEdge::TYPE_BOULEVARD:
			height = avenueHeight;
			break;
		case RoadEdge::TYPE_AVENUE:
			height = avenueHeight;
			break;
		case RoadEdge::TYPE_STREET:
			height = avenueHeight;
			break;
		}

		color = graph[*ei]->color;
		bgColor = graph[*ei]->bgColor;

		// グループに基づいて色を決定
		/*
		switch (graph[*ei]->shapeType) {
		case RoadEdge::SHAPE_DEFAULT:
			color = QColor(255, 255, 255);
			break;
		case RoadEdge::SHAPE_GRID:	// grid
			color = QColor(255 * (1.0f - graph[*ei]->gridness), 255 * (1.0f - graph[*ei]->gridness), 255);
			break;
		case RoadEdge::SHAPE_RADIAL:	// radial
			color = QColor(0, 255, 0);
			break;
		case RoadEdge::SHAPE_PLAZA: // plaza
			color = QColor(255, 0, 0);
			break;
		default:
			color = QColor(255, 255, 255);
			break;
		}
		*/

		// draw the border of the road segment
		if ((showHighways && edge->type == RoadEdge::TYPE_HIGHWAY) || (showBoulevard && edge->type ==  RoadEdge::TYPE_BOULEVARD) || (showAvenues && edge->type ==  RoadEdge::TYPE_AVENUE) || (showLocalStreets && edge->type ==  RoadEdge::TYPE_STREET)) {
			addMeshFromEdge(renderables[0], edge, widthBase * (1.0f + curbRatio), bgColor, 0.0f);
			addMeshFromEdge(renderables[0], edge, widthBase, color, height);
		}
	}

	// road vertex
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
		if (!graph[*vi]->valid) continue;

		if (graph[*vi]->seed) {
			addMeshFromVertex(renderables[1], graph[*vi], QColor(0, 0, 255), 2.0f);
		}
	}

	modified = false;
}

/**
 * Add a mesh for the specified edge.
 */
void RoadGraph::addMeshFromEdge(RenderablePtr renderable, RoadEdgePtr edge, float widthBase, QColor color, float height) {
	Vertex v;

	// define the width of the road segment
	float width;
	switch (edge->type) {
	case RoadEdge::TYPE_HIGHWAY:
		width = widthBase * 2.0f;
		break;
	case RoadEdge::TYPE_BOULEVARD:
	case RoadEdge::TYPE_AVENUE:
		width = widthBase * 1.5f;
		break;
	case RoadEdge::TYPE_STREET:
		width = widthBase * 1.0f;
		break;
	}

	int num = edge->polyLine.size();

	// draw the edge
	for (int i = 0; i < num - 1; ++i) {
		QVector2D pt1 = edge->polyLine[i];
		QVector2D pt2 = edge->polyLine[i + 1];
		QVector2D vec = pt2 - pt1;
		vec = QVector2D(-vec.y(), vec.x());
		vec.normalize();

		QVector2D p0 = pt1 + vec * width * 0.5f;
		QVector2D p1 = pt1 - vec * width * 0.5f;
		QVector2D p2 = pt2 - vec * width * 0.5f;
		QVector2D p3 = pt2 + vec * width * 0.5f;

		v.color[0] = color.redF();
		v.color[1] = color.greenF();
		v.color[2] = color.blueF();
		v.color[3] = color.alphaF();
		v.normal[0] = 0.0f;
		v.normal[1] = 0.0f;
		v.normal[2] = 1.0f;

		v.location[2] = height;

		v.location[0] = p0.x();
		v.location[1] = p0.y();
		renderable->vertices.push_back(v);

		v.location[0] = p1.x();
		v.location[1] = p1.y();
		renderable->vertices.push_back(v);

		v.location[0] = p2.x();
		v.location[1] = p2.y();
		renderable->vertices.push_back(v);

		v.location[0] = p0.x();
		v.location[1] = p0.y();
		renderable->vertices.push_back(v);

		v.location[0] = p2.x();
		v.location[1] = p2.y();
		renderable->vertices.push_back(v);

		v.location[0] = p3.x();
		v.location[1] = p3.y();
		renderable->vertices.push_back(v);
	}
}

/**
 * Add a mesh for the specified edge.
 */
void RoadGraph::addMeshFromVertex(RenderablePtr renderable, RoadVertexPtr vertex, QColor color, float height) {
	Vertex v;

	// draw the vertex
	v.color[0] = color.redF();
	v.color[1] = color.greenF();
	v.color[2] = color.blueF();
	v.color[3] = color.alphaF();
	v.normal[0] = 0.0f;
	v.normal[1] = 0.0f;
	v.normal[2] = 1.0f;

	v.location[0] = vertex->pt.x();
	v.location[1] = vertex->pt.y();
	v.location[2] = height;

	renderable->vertices.push_back(v);
}

bool RoadGraph::getModified() {
	return modified;
}

void RoadGraph::setModified() {
	modified = true;
}

void RoadGraph::clear() {
	graph.clear();

	modified = true;
}

void RoadGraph::setZ(float z) {
	// define the width per lane
	float widthBase2;
	if (z < 300.0f) {
		widthBase2 = 2.0f;
	} else if (z < 600.0f) {
		widthBase2 = 4.0f;
	} else if (z < 1080.0f) {
		widthBase2 = 10.0f;
	} else if (z < 5760.0f) {
		widthBase2 = 12.0f;
	} else {
		widthBase2 = 24.0f;
	}
	if (widthBase != widthBase2) {
		widthBase = widthBase2;
		modified = true;
	}

	// define the curb ratio
	float curbRatio2;
	if (z < 2880.0f) {
		curbRatio2 = 0.4f;
	} else {
		curbRatio2 = 0.8f;
	}
	if (curbRatio != curbRatio2) {
		curbRatio = curbRatio2;
		modified = true;
	}

	// define the height
	float highwayHeight2 = (float)((int)(z * 0.012f)) * 0.1f;
	if (highwayHeight != highwayHeight2) {
		highwayHeight = highwayHeight2;
		avenueHeight = highwayHeight2 * 0.66f;
		modified = true;
	}
}
