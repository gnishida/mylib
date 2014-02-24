#include "RoadAreaSet.h"
#include "GraphUtil.h"

const size_t RoadAreaSet::size() const {
	return areas.size();
}

RoadArea& RoadAreaSet::operator[](int index) {
	return areas[index];
}

void RoadAreaSet::add(const RoadArea &area) {
	areas.push_back(area);
}

void RoadAreaSet::clear() {
	roads.clear();
	areas.clear();
}

void RoadAreaSet::remove(int index) {
	areas.erase(areas.begin() + index);
}

void RoadAreaSet::setZ(float z) {
	roads.setZ(z);
	for (int i = 0; i < areas.size(); ++i) {
		areas[i].roads.setZ(z);
	}
}

void RoadAreaSet::addRoads(int roadType, int lanes, bool oneWay, const Polyline2D &polyline) {
	GraphUtil::addEdge(roads, polyline, roadType, lanes, oneWay);
	GraphUtil::planarify(roads);
	GraphUtil::clean(roads);
}

void RoadAreaSet::mergeRoads() {
	for (int i = 0; i < areas.size(); ++i) {
		GraphUtil::mergeRoads(roads, areas[i].roads);
		areas[i].roads.clear();
	}
}

/**
 * 与えられたfeatureノード配下のXML情報に基づいて、グリッド特徴量を設定する。
 */
void RoadAreaSet::load(QString filename) {
	// ファイル名からディレクトリ部を取得
	QString dirname;
	int index = filename.lastIndexOf("/");
	if (index > 0) {
		dirname = filename.mid(0, index);
	}

	QFile file(filename);

	QDomDocument doc;
	doc.setContent(&file, true);
	QDomElement root = doc.documentElement();

	if (root.hasAttribute("roads")) {
		QString roadFilename = dirname + "/" + root.attribute("roads");
		GraphUtil::loadRoads(roads, roadFilename);
	}

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "area") {
			RoadArea area;
			area.load(node);
			areas.push_back(area);
		}

		node = node.nextSibling();
	}
}

void RoadAreaSet::save(QString filename) {
	// ファイル名からファイル名を取得
	QString name;
	int index = filename.lastIndexOf("/");
	if (index > 0) {
		name = filename.mid(0, index);
	} else {
		name = filename;
	}

	QDomDocument doc;

	QString roadFilename = name + ".gsm";
	GraphUtil::saveRoads(roads, roadFilename);

	QDomElement root = doc.createElement("areas");
	root.setAttribute("roads", roadFilename);
	doc.appendChild(root);

	for (int i = 0; i < areas.size(); ++i) {
		areas[i].save(doc, root);
	}

	// write the dom to the file
	QFile file(filename);
	file.open(QIODevice::WriteOnly);

	QTextStream out(&file);
	doc.save(out, 4);
}

