﻿#pragma once

#include <vector>
#include <QVector2D>
#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "BBox.h"

/**
 * QVector2DをBoostのpointの代替として使用
 */
// BBox.hで定義済みなので、再定義は不要
//BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(QVector2D, float, boost::geometry::cs::cartesian, x, y, setX, setY)



class Polygon2D : public std::vector<QVector2D> {
public:
	Polygon2D() {}
	~Polygon2D() {}

	void correct();
	float area() const;
	QVector2D centroid() const;
	bool contains(const QVector2D &pt) const;
	bool contains(const QVector2D &pt);
	Polygon2D convexHull() const;
	BBox envelope() const;
	bool intersects(const QVector2D& a, const QVector2D& b, QVector2D& intPt) const;
	void translate(float x, float y);
	void translate(float x, float y, Polygon2D &ret) const;
	void rotate(float angle);
	void rotate(float angle, Polygon2D &ret) const;

	std::vector<Polygon2D> tessellate();
	QVector2D getOBB(Polygon2D &obb) const;
	QVector2D getOBB(const QVector2D& dir, Polygon2D& obb) const;
};

/**
 * Polygon2DをBoostのringの代替として使用
 */
BOOST_GEOMETRY_REGISTER_RING(Polygon2D)

