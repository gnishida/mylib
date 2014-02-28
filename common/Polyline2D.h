﻿#pragma once

#include <QVector2D>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/linestring.hpp>

class Polyline2D : public std::vector<QVector2D> {
public:
	Polyline2D() {}
	~Polyline2D() {}

	//void translate(float x, float y);
	const QVector2D & last() const;
	QVector2D & last();

	void rotate(float angle, const QVector2D &orig);
};

/**
 * Linestringを定義
 */
BOOST_GEOMETRY_REGISTER_LINESTRING(Polyline2D)
