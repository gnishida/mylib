#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include "Polyline2D.h"

/**
 * 当該ポリラインを移動する。
 */
/*void Polyline2D::translate(float x, float y) {
	Polyline2D temp = *this;
	this->clear();

	boost::geometry::strategy::transform::translate_transformer<QVector2D, QVector2D> translate(x, y);
    boost::geometry::transform(temp, *this, translate);
}*/
