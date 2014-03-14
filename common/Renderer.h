#pragma once

#include <vector>
#include <QVector2D>
#include "common.h"
#include "

class Renderer {
public:
	QVector2D minPt, maxPt;

public:
	BBox() {}
	~BBox() {}

	void addPoint(const QVector2D& pt);
	bool contains(const QVector2D& pt) const;
	QVector2D midPt() const;
	float dx() const;
	float dy() const;
	float area() const;
};

