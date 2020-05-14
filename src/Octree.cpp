//  Justin Magadia 5/13/2020


#include "ofApp.h"
#include "Util.h"
#include "Octree.h"
#include "vector3.h"

// draw Octree (recursively)
//
void Octree::draw(TreeNode & node, int numLevels, int level) {




	drawBox(node.box);
	//cout << "DRAWN " << level << endl;
	level++;
	for (int i = 0; i < node.children.size(); i++) {
		draw(node.children[i], numLevels, level);

	}
}



// draw only leaf Nodes
//
void Octree::drawLeafNodes(TreeNode & node) {
	if (node.children.size() <= 0) {
		drawBox(node.box);
		return;
	}

	for (int i = 0; i < node.children.size(); i++) {
		drawLeafNodes(node.children[i]);
	}
}


//draw a box from a "Box" class  
//
void Octree::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box Octree::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	cout << "vertices: " << n << endl;
	//	cout << "min: " << min << "max: " << max << endl;
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

// getMeshPointsInBox:  return an array of indices to points in mesh that are contained 
//                      inside the Box.  Return count of points found;
//
int Octree::getMeshPointsInBox(const ofMesh & mesh, const vector<int>& points,
	Box & box, vector<int> & pointsRtn)
{
	int count = 0;
	for (int i = 0; i < points.size(); i++) {
		ofVec3f v = mesh.getVertex(points[i]);
		if (box.inside(Vector3(v.x, v.y, v.z))) {
			count++;
			pointsRtn.push_back(points[i]);
		}
	}
	return count;
}



//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void Octree::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

void Octree::create(const ofMesh & geo, int numLevels) {
	// initialize octree structure
	mesh = geo;
	numOfLevels = numLevels;
	root.box = meshBounds(mesh);
	root.points = getIntIndices(mesh);
	//cout << "created " << root.points.size() << " " << mesh.getNumVertices() << " " << mesh.getNumIndices()<< endl;
	subdivide(mesh, root, numOfLevels, 0);
}

vector<int> Octree::getIntIndices(const ofMesh & mesh) {
	vector<ofIndexType> oldIndices;
	oldIndices = mesh.getIndices();
	//cout << "pointsizee:  " << oldIndices.size() << endl;

	vector<int> theIndices(oldIndices.size());

	for (int i = 0; i < oldIndices.size(); i++) {
		theIndices[i] = oldIndices[i];
	}
	return theIndices;
}

void Octree::subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level) {

	if (level >= numLevels) return;
	level++;
	//cout << "Running " << level << endl;
	//Create a vector with all 8 children boxes
	vector<Box> childBoxes8;
	subDivideBox8(node.box, childBoxes8);
	for (int i = 0; i < 8; i++) {
		TreeNode child;
		child.box = childBoxes8[i];
		getMeshPointsInBox(mesh, node.points, childBoxes8[i], child.points);

		//only subdivide child if it has more than one point inside
		if (child.points.size() > 1) {
			//Divide the child to 8 boxes
			subdivide(mesh, child, numLevels, level);
		}

		if (child.points.size() > 0) {
			//Add the child to the child nodes array if it is not empty
			//cout << "Points: " <<child.points.size() << " level: " << level << endl;
			node.children.push_back(child);
		}
	}

}

bool Octree::intersect(const Ray &ray, const TreeNode & node, TreeNode & nodeRtn) {

	//See if it intersects
	if (node.box.intersect(ray, -10000, 10000)) {

		//if it’s a leaf node, return it
		if (node.children.size() == 0) {
			nodeRtn = node;
			return true;
		}
		else {
			//Look deeper if it’s not a leaf node
			for (int i = 0; i < node.children.size(); i++) {
				//if (node.children[i].box.intersect(ray, -10000, 10000)) {
				intersect(ray, node.children[i], nodeRtn);
				//}
			}
		}
	}
	else {
		return false;
	}

}

bool Octree::pointIntersect(ofVec3f &point, TreeNode & node, TreeNode & nodeRtn) {

	//See if the point intersects with the box
	if (node.box.pointInside(point)) {

		//if it’s a leaf node, return it
		if (node.children.size() == 0) {
			nodeRtn = node;
			return true;
		}
		else {
			//Look deeper if it’s not a leaf node
			for (int i = 0; i < node.children.size(); i++) {
				pointIntersect(point, node.children[i], nodeRtn);
			}
		}
	}
	else {
		return false;
	}

}
