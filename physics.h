#pragma GCC optimize("Ofast")

#include <iostream>
#include <vector>
#include <string>
#include <variant>
#include <algorithm>
#include <iterator>
#include <thread>
#include <windows.h>           // for windows
#include "SFML/Graphics.hpp"
#include "LineSegmentIntersection.h"
#include "FalconEngine.h"

namespace bridgeMechanics {

	using namespace std;

	#define NODE_DIAMETER 7 // in pixels
	#define LINE_THICKNESS 2 // in pixels
	#define PIXELS_PER_UNIT 15

	namespace debug {
		std::vector<sf::Vector2f> debugPoints;
		void addDebugPoint(sf::Vector2f pos) {
			debugPoints.push_back(pos);
		}
		void clearDebugPoints() {
			debugPoints.clear();
		}

		void drawDebugPoints() {
			// debug points

			static int last = 0;
			
			if (last < debug::debugPoints.size()) {
				for (unsigned int i = 0; i < last; i++) {

					sf::Vector2f pos = debug::debugPoints[i];
					pos.x *= PIXELS_PER_UNIT;
					pos.y *= PIXELS_PER_UNIT;

					sf::CircleShape c = sf::CircleShape(NODE_DIAMETER*.5);
					c.setOrigin(c.getRadius(), c.getRadius());
					c.setPosition(pos);
					c.setFillColor(sf::Color(255,10,10,255));
					windowManager.RT->draw(c);
				}
				++last;
			}
		}
	}

	void evenSubdivide() {

	}

	struct node {
		sf::Vector2f velocity = sf::Vector2f(0, 0);
		sf::Vector2f position = sf::Vector2f(0, 0);
		std::vector<node*> neighboringVertices;
		std::vector<std::vector<node*>> corners;
		std::vector<float> originalAngles;
		bool updated = false;

		void addImpulse(const sf::Vector2f impulse) {
			velocity += impulse;
		}

		void rotateAroundPoint(const sf::Vector2f origin, float degrees) {
			// convert to radians
			const float angle = degrees * 0.0174533;
			float s = sin(angle);
			float c = cos(angle);
			
			sf::Vector2f p = position;
			// translate point back to origin:
			p.x -= origin.x;
			p.y -= origin.y;

			// rotate point
			float xnew = p.x * c - p.y * s;
			float ynew = p.x * s + p.y * c;

			// translate point back:
			p.x = xnew + origin.x;
			p.y = ynew + origin.y;
			position = p;
		}
	};

	struct handle {
		int index = 0;
		bool clear = false;
	};


	node* pointToTest = NULL;




	struct edge {
		node* n1 = NULL;
		node* n2 = NULL;
		float originalLength = 0;
		float originalAngle = 0;
		int stiffness = 5;
		float strength = 1;
		sf::Vector2f getNormal() {
			if (n1 == NULL || n2 == NULL) {
				perror("N1 or N2 are NULL, cannot get the vector of the edge!");
				return sf::Vector2f(0, 0);
			}
			sf::Vector2f temp = n1->position - n2->position;
			sf::Vector2f Normal = sf::Vector2f(temp.y, temp.x);
			Normal = sf::Vector2f(n1->position.y - n2->position.y, n2->position.x - n1->position.x);
			Normal = util::normalize(Normal);
			return Normal;
		}

		inline float getAngleDeg() {
			if (n1 == NULL || n2 == NULL) {
				perror("N1 or N2 are NULL, cannot get the angle of the edge!");
				return 0;
			}
			float rotation = atan2(n1->position.y - n2->position.y, n1->position.x - n2->position.x);
			// converts radians to degrees
			rotation = rotation * 180 / 3.14159;
			return rotation;
		}

		bool colinear(edge &other) {
			if (n1 != NULL && n2 != NULL) {
				// checks alignment
				if (util::crossProduct(n1->position,n2->position) * util::crossProduct(other.n1->position,other.n2->position) == 0) {
					return true;
				}
			}
			return false;
		}

		bool liesOnEdge(node &n, float tolerance = .05) {
			sf::Vector2f normal = this->getNormal();
			sf::Vector2f normalInverse = this->getNormal();
			normal.x *= tolerance;
			normal.y *= tolerance;
			normalInverse.x *= -1 * tolerance;
			normalInverse.y *= -1 * tolerance;

			normal += n.position;
			normalInverse += n.position;
			//std::cout << "(" << point->position.x << "," << point->position.y << ")";
			//drawDotAtPos(pointToTest->position, RT);

			if (doIntersect(normal, normalInverse, n1->position, n2->position)) {
				return true;
			}

			return false;
		}
		//https://stackoverflow.com/questions/26849632/see-if-a-point-lies-on-a-linevector
		bool isPointOnLine(sf::Vector2f p, float tolerance = .00001) // returns true if p is on line n1, n2
		{
			sf::Vector2f va = n1->position - n2->position;
			sf::Vector2f vb = p - n2->position;
			float area = va.x * vb.y - va.y * vb.x;
			if (abs(area) < tolerance) {
				if (p.x >= std::min(n1->position.x, n2->position.x) && p.x <= std::max(n1->position.x, n2->position.x)) {
					if (p.y >= std::min(n1->position.y, n2->position.y) && p.y <= std::max(n1->position.y, n2->position.y)) {
						return true;
					}
				}
			}
			return false;
		}
	};

	inline void drawDotAtPos(sf::Vector2f pos, sf::RenderTexture& RT, sf::Color color = sf::Color::Red) {
		sf::CircleShape c = sf::CircleShape(5);
		c.setFillColor(color);
		c.setOrigin(sf::Vector2f(c.getRadius(), c.getRadius()));
		c.setPosition(pos.x * PIXELS_PER_UNIT, pos.y * PIXELS_PER_UNIT);
		RT.draw(c);
	}

	inline void drawLine(sf::Vector2f p1, sf::Vector2f p2, sf::RenderTexture &RT, sf::Color color = sf::Color::Green) {
		register sf::RectangleShape rect;
		float length = util::distance(p1.x, p2.x, p1.y, p2.y) * PIXELS_PER_UNIT;
		rect.setFillColor(color);
		rect.setSize(sf::Vector2f(LINE_THICKNESS, length));
		rect.setOrigin(sf::Vector2f(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2));
		register sf::Vector2f midpoint = p1 + p2;
		midpoint.x /= 2;
		midpoint.y /= 2;
		midpoint.x *= PIXELS_PER_UNIT;
		midpoint.y *= PIXELS_PER_UNIT;
		float rotation = atan2(p1.y - p2.y, p1.x - p2.x);
		// converts radians to degrees
		rotation = rotation * 180 / 3.14159;
		rect.setPosition(midpoint);
		rect.setRotation(rotation + 90);

		RT.draw(rect);
	}

	inline bool isPointWithinShape(vector<edge> edges, sf::Vector2f point) {
		int intersectionCount = 0;

		sf::Vector2f raycastBegin = point;
		sf::Vector2f raycastEnd = sf::Vector2f(999999, 0);
		for (unsigned int i = 0; i < edges.size(); ++i) {
			if (doIntersect(edges[i].n1->position, edges[i].n2->position, raycastBegin, raycastEnd))
			{
				++intersectionCount;
			}
		}

		if (intersectionCount % 2 == 1) {
			return true;
		}
		return false;
	}

	/*
	inline bool isPointOnLine(edge& e, sf::Vector2f point) {
		float x = point.x - e.n1->position.x;
		float y = point.y - e.n1->position.y;
		float diffX = e.n1->position.x - e.n2->position.x;
		float diffY = e.n1->position.y - e.n2->position.y;
		float slope = diffX / diffY;

		// If (x, y) satisfies the equation of the line

		if (y == ((slope * x) + e.n1->position.y)) {
			return true;
		}

		return false;
	}*/

	struct bridge {
		vector<node> nodes;
		vector<edge> edges;
		std::vector<int>::iterator it;
		vector<edge> outline;
		vector<node> outlineNodes;
		vector<handle> handles;
		
		void init(int maxNodes = 1024) {
			nodes.reserve(maxNodes);
			edges.reserve(maxNodes * 2);
		}


		handle& addNode(node n) {
			// attemps to find a clear handle
			for (unsigned int i = 0; i < handles.size(); ++i) {
				handle& hdl = handles[i];
				if (hdl.clear) {
					nodes.push_back(n);
					hdl.index = nodes.size() - 1;
					hdl.clear = false;
					return hdl;
				}
			}

			// if no clear handles were found
			nodes.push_back(n);
			handle hdl;
			hdl.index = nodes.size() - 1;
			handles.push_back(hdl);
			return hdl;
		}

		void removeNode(handle& hdl) {
			nodes.erase(nodes.begin() + hdl.index);
			hdl.clear = true;
			// decrease the index of all other handles that are greater than the arg hdl.index
			for (unsigned int i = 0; i < handles.size(); ++i) {
				if (handles[i].index > hdl.index) {
					--handles[i].index;
				}
			}
		}

		vector<edge*> getConnectedEdges(node* n) {
			vector<edge*> connected;
			for (unsigned int i = 0; i < edges.size(); i++) {
				edge& e = edges[i];
				if (e.n1 == n) {
					connected.push_back(&e);
				}
				else if (e.n2 == n) {
					connected.push_back(&e);
				}
			}
			return connected;
		}

		void mergeNodesAndEdges(float threshold = 0.01) {
			std::cout << "SIZE OF NODES: " << nodes.size() << "\n";
			std::cout << "SIZE OF EDGES: " << edges.size() << "\n";
			/*vector<node*> nodesToMerge;
			sf::Vector2f avgPos = sf::Vector2f(0, 0);
			for (unsigned int i = 0; i < nodes.size(); ++i) {
				node& n1 = nodes[i];
				sf::Vector2f p1 = n1.position;
				for (unsigned int j = 0; j < nodes.size(); ++j) {
					if (i != j) {
						node& n2 = nodes[j];
						sf::Vector2f p2 = n2.position;
						if (util::distance(p1.x, p2.x, p1.y, p2.y) < threshold) {
							vector<edge*> connectedEdges = getConnectedEdges(&n1);
							for (unsigned int k = 0; k < connectedEdges.size(); ++k) {
								edge& e = *connectedEdges[k];
								if (e.n1 == &n2) {
									nodesToMerge.push_back(&n2);
									avgPos += n2.position;
								}
								else if (e.n2 == &n2) {
									nodesToMerge.push_back(&n2);
									avgPos += n2.position;
								}

							}
						}
					}
				}
			}

			// calculates average pos
			avgPos.x /= nodesToMerge.size();
			avgPos.y /= nodesToMerge.size();


			// sets all the nodes to merge to the first index of nodes to merge
			nodesToMerge[0]->position = avgPos;
			for (unsigned int i = 1; i < nodesToMerge.size(); i++) {
				nodesToMerge[i] = nodesToMerge[0];
			}*/

			vector<vector<node*>> nodesToMerge;
			vector<sf::Vector2f> usedPositions;
			static vector<node> newNodes;
			newNodes.reserve(4096);


			for (unsigned int i = 0; i < nodes.size(); ++i) {
				node &n = nodes[i];

				bool isUsed = false;
				for (unsigned int p = 0; p < usedPositions.size(); ++p) {
					if (n.position == usedPositions[p]) {
						isUsed = true;
					}
				}
				if (!isUsed) {
					//nodeGroup.push_back(&n2);
					newNodes.push_back(n);
					usedPositions.push_back(n.position);
				}
				//nodesToMerge.push_back(nodeGroup);
			}



			struct twoVec {
				sf::Vector2f p1;
				sf::Vector2f p2;
			};
			vector<twoVec> usedEdges;
			vector<twoVec> oldEdges;
			for (unsigned int i = 0; i < edges.size(); i++) {
				twoVec v2;
				v2.p1 = edges[i].n1->position;
				v2.p2 = edges[i].n2->position;
				oldEdges.push_back(v2);
			}



			nodes = newNodes;

			vector<edge> newEdges;
			newEdges.reserve(4096);
		

			// needs optimization
			for (unsigned int i = 0; i < oldEdges.size(); ++i) {
				twoVec e = oldEdges[i];
				edge newE;
				bool used = false;
				for (unsigned int j = 0; j < nodes.size(); ++j) {
					
					if (e.p1 == nodes[j].position) {
						newE.n1 = &nodes[j];
					}
					if (e.p2 == nodes[j].position) {
						newE.n2 = &nodes[j];
					}
				}

				if (newE.n1 != NULL && newE.n2 != NULL) {
					for (unsigned int j = 0; j < newEdges.size(); j++) {
						edge& e = newEdges[j];
						if ((e.n1->position == newE.n1->position && e.n2->position == newE.n2->position) ||
							(e.n2->position == newE.n1->position && e.n1->position == newE.n2->position)) {
							used = true;
							break;
						}
					}
					if (!used) {
						newE.originalLength = util::distance(newE.n1->position, newE.n2->position);
						newE.originalAngle = newE.getAngleDeg();
						newEdges.push_back(newE);
					}
				}
			}
			edges = newEdges;
			


			std::cout << "SIZE OF NODES: " << nodes.size() << "\n";
			std::cout << "SIZE OF EDGES: " << edges.size() << "\n";
		}

		node* getClosestNode(sf::Vector2f pos, float threshold = 2) {
			node* closest = NULL;
			float cloestDist = 9999999999999;
			for (unsigned int i = 0; i < nodes.size(); i++) {
				node& n = nodes[i];
				float dist = util::distance(pos.x, n.position.x, pos.y, n.position.y);
				if (dist < cloestDist && dist < threshold) {
					closest = &n;
					cloestDist = dist;
				}
			}

			if (NULL == closest)
			{ std::cout << "Closest node not found";
				return closest; }
			else
			{ return closest; }
		}
	} b;

	void isWithinShape(vector<sf::Vector2f> &points, sf::Vector2f pos) {

	}

	void createBridge(vector<sf::Vector2f> points) {
		// simple outline
		b.nodes.clear();
		b.edges.clear();
		b.nodes.reserve(10000);
		b.edges.reserve(10000);
		b.outlineNodes.reserve(1024);
		int leftLimit = 999999999;
		int rightLimit = -999999999;
		int lowerLimit = 999999999;
		int upperLimit = -999999999;

		for (unsigned int i = 0; i < points.size(); ++i) {
			node n1;
			n1.position = points[i];
			b.outlineNodes.push_back(n1);
			node n2;
			n2.position = points[(i + 1)%points.size()];
			b.outlineNodes.push_back(n2);
			edge e;
			e.n1 = &b.outlineNodes[b.outlineNodes.size() - 2];
			e.n2 = &b.outlineNodes[b.outlineNodes.size() - 1];
			e.originalLength = util::distance(e.n1->position, e.n2->position);
			b.outline.push_back(e);
		}

		// calculates bounds
		for (unsigned int i = 0; i < b.outlineNodes.size(); i++) {
			node& n = b.outlineNodes[i];
			if (n.position.x < leftLimit) {
				leftLimit = n.position.x;
			}
			if (n.position.x > rightLimit) {
				rightLimit = n.position.x;
			}

			if (n.position.y < lowerLimit) {
				lowerLimit = n.position.y;
			}
			if (n.position.y > upperLimit) {
				upperLimit = n.position.y;
			}
		}

		// creates initial grid
		for (unsigned int y = lowerLimit; y <= upperLimit; ++y) {
			for (unsigned int x = leftLimit; x <= rightLimit; ++x) {
				const sf::Vector2f pos = sf::Vector2f(x,y);
				bool liesOnAnEdge = false;
				for (unsigned int i = 0; i < b.outline.size(); ++i) {
					edge& e = b.outline[i];
					if (e.isPointOnLine(pos)) {
						liesOnAnEdge = true;
					}
				}
				if (isPointWithinShape(b.outline, pos) || liesOnAnEdge) {
					node n;
					n.position = pos;
					b.nodes.push_back(n);

					// connects nodes horizontally
					if (x != leftLimit) {
						edge e;
						e.n1 = &b.nodes[b.nodes.size() - 2];
						e.n2 = &b.nodes[b.nodes.size() - 1];
						e.originalAngle = e.getAngleDeg();
						e.originalLength = util::distance(e.n1->position, e.n2->position);
						b.edges.push_back(e);
					}

					// connects to rows above
					for (unsigned int i = 0; i < b.nodes.size(); ++i) {
						node& other = b.nodes[i];
						if (other.position.y == pos.y - 1 && other.position.x == pos.x) {
							edge e;
							e.n1 = &b.nodes[b.nodes.size() - 1];
							e.n2 = &other;
							e.originalAngle = e.getAngleDeg();
							e.originalLength = util::distance(e.n1->position, e.n2->position);
							b.edges.push_back(e);
						}
					}

					// connects diagonally

					for (unsigned int i = 0; i < b.nodes.size(); ++i) {
						node& other = b.nodes[i];
						if (other.position.y == pos.y - 1 && other.position.x == pos.x - 1) {
							edge e;
							e.n1 = &b.nodes[b.nodes.size() - 1];
							e.n2 = &other;
							e.originalAngle = e.getAngleDeg();
							e.originalLength = util::distance(e.n1->position, e.n2->position);
							b.edges.push_back(e);
						}
						if (other.position.y == pos.y - 1 && other.position.x == pos.x + 1) {
							edge e;
							e.n1 = &b.nodes[b.nodes.size() - 1];
							e.n2 = &other;
							e.originalAngle = e.getAngleDeg();
							e.originalLength = util::distance(e.n1->position, e.n2->position);
							b.edges.push_back(e);
						}
					}
				}
			}
		}

		vector<edge*> bufferEdges;

		// fills in angle gaps PT.1
		for (unsigned int i = 0; i < b.outline.size(); ++i) {
			edge& outlineE = b.outline[i];
			node* last = NULL;
			for (unsigned int j = 0; j < b.nodes.size(); ++j) {
				node& n = b.nodes[j];
				if (outlineE.isPointOnLine(n.position)) {
					if (last != NULL) {
						edge newE;
						newE.n1 = &n;
						newE.n2 = last;
						newE.originalLength = util::distance(newE.n1->position, newE.n2->position);
						newE.originalAngle = newE.getAngleDeg();
						if (fmod(newE.getAngleDeg(),90) != 0) {
							b.edges.push_back(newE);
							bufferEdges.push_back(&b.edges[b.edges.size()-1]);
						}
					}
					last = &n;
				}
			}
		}

		// fills in angle gaps PT.2
		vector<node> oldNodes = b.nodes;
		for (unsigned int i = 0; i < b.outline.size(); ++i) {
			edge& outlineE = b.outline[i];
			node* last = NULL;
			for (unsigned int j = 0; j < oldNodes.size(); ++j) {
				node& n = oldNodes[j];
				sf::Vector2f raycastBegin = n.position;
				sf::Vector2f raycastEnd;
				raycastEnd = raycastBegin + sf::Vector2f(-1, 0);
				if (doIntersect(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd))
				{
					node n2;
					n2.position = CalcIntersection(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd);
					b.nodes.push_back(n2);
				}
				raycastEnd = raycastBegin + sf::Vector2f(1, 0);
				if (doIntersect(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd))
				{
					node n2;
					n2.position = CalcIntersection(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd);
					b.nodes.push_back(n2);
				}
				raycastEnd = raycastBegin + sf::Vector2f(0, -1);
				if (doIntersect(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd))
				{
					node n2;
					n2.position = CalcIntersection(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd);
					b.nodes.push_back(n2);
				}
				raycastEnd = raycastBegin + sf::Vector2f(0, 1);
				if (doIntersect(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd))
				{
					node n2;
					n2.position = CalcIntersection(outlineE.n1->position, outlineE.n2->position, raycastBegin, raycastEnd);
					b.nodes.push_back(n2);

					edge newE;

					newE.n1 = &b.nodes[j];
					newE.n2 = &b.nodes[b.nodes.size() - 1];
					newE.originalLength = util::distance(newE.n1->position, newE.n2->position);
					//b.edges.push_back(newE);

				}
			}
		}

		b.mergeNodesAndEdges();

		for (unsigned int i = 0; i < b.nodes.size(); ++i) {
			node& n1 = b.nodes[i];
			sf::Vector2f p1 = n1.position;
			std::vector<node*> leftUp;
			std::vector<node*> leftDown;
			std::vector<node*> rightUp;
			std::vector<node*> rightDown;

			leftUp.push_back(&n1);
			leftDown.push_back(&n1);
			rightUp.push_back(&n1);
			rightDown.push_back(&n1);
			

			for (unsigned int j = 0; j < b.nodes.size(); ++j) {
				if (i != j) {
					node& n2 = b.nodes[j];
					sf::Vector2f p2 = n2.position;
					//if (util::distance(n1.position, n2.position) < 10) {
					//	n1.neighboringVertices.push_back(&n2);
					//}
					// LEFT UP
					if (p2.x == p1.x - 1 || p2.y == p1.y - 1) {
						leftUp.push_back(&n2);
					}
					// LEFT DOWN
					if (p2.x == p1.x - 1 || p2.y == p1.y + 1) {
						leftDown.push_back(&n2);
					}
					// RIGHT UP
					if (p2.x == p1.x + 1 || p2.y == p1.y - 1) {
						rightUp.push_back(&n2);
					}
					// RIGHT DOWN
					if (p2.x == p1.x + 1 || p2.y == p1.y + 1) {
						rightDown.push_back(&n2);
					}
				}
			}

			if (leftUp.size() == 3) {
				n1.originalAngles.push_back(util::angleBetweenThreePoints(
					leftUp[1]->position, leftUp[0]->position, leftUp[2]->position
				));
				n1.corners.push_back(leftUp);
			}
			if (leftDown.size() == 3) {
				n1.originalAngles.push_back(util::angleBetweenThreePoints(
					leftDown[1]->position, leftDown[0]->position, leftDown[2]->position
				));
				n1.corners.push_back(leftDown);
			}
			if (rightUp.size() == 3) {
				n1.originalAngles.push_back(util::angleBetweenThreePoints(
					rightUp[1]->position, rightUp[0]->position, rightUp[2]->position
				));
				n1.corners.push_back(rightUp);
			}
			if (rightDown.size() == 3) {
				n1.originalAngles.push_back(util::angleBetweenThreePoints(
					rightDown[1]->position, rightDown[0]->position, rightDown[2]->position
				));
				n1.corners.push_back(rightDown);
			}
		}
	}

	// input handling
	namespace inputHandling {
		static node* currentNode = NULL;
		bool mouseDown = false;
		void onMouseClick(sf::Vector2i mPos) {
			currentNode = b.getClosestNode(sf::Vector2f(float(mPos.x) / PIXELS_PER_UNIT, float(mPos.y) / PIXELS_PER_UNIT));
			mouseDown = true;
		}
		void handleMouseDown(sf::Vector2i mPos) {
			if (currentNode != NULL) {
				currentNode->position = sf::Vector2f(float(mPos.x) / PIXELS_PER_UNIT, float(mPos.y) / PIXELS_PER_UNIT);
			}
		}
		void onMouseRelease() {
			currentNode = NULL;
			mouseDown = true;
		}
	}

	void update() {
		//b.nodes[790].position += sf::Vector2f(-3, 3);


		register float diff = 0, dist = 0;
		sf::Vector2f v1v2;
		for (unsigned int j = 0; j < 100; ++j) {

			for (unsigned int i = 0; i < b.edges.size(); i++) {
				edge& e = b.edges[i];
				if (j <= e.stiffness) {
					dist = 0;
					v1v2 = e.n2->position - e.n1->position;
					dist = util::distance(e.n1->position, e.n2->position);

					if (dist != 0) { // prevents divide by zero error
						//float diff = (e.originalLength - dist) / dist;
						diff = (e.originalLength - dist);

						v1v2 = util::normalize(v1v2);
						e.n1->position -= v1v2 * diff * .5f;
						//e.n1->addImpulse(v1v2 * diff * .5f);
						//e.n1->updated = true;
						e.n2->position += v1v2 * diff * .5f;
						//e.n2->addImpulse(v1v2 * diff * -.5f);
						//e.n2->updated = true;
					}
					if (dist > e.originalLength + 55.8) {
						b.edges.erase(b.edges.begin() + i);
					}
				}
			}
		}
		const float tmp = 1.4;
		for (unsigned int i = 0; i < b.nodes.size(); ++i) {
			node& n = b.nodes[i];
			//n.position += n.velocity;
			//n.velocity *= .1f;
			for (unsigned int j = 0; j < n.neighboringVertices.size(); ++j) {
				node& n2 = *n.neighboringVertices[j];
				dist = util::distance(n.position, n2.position);
				if (dist < tmp) {
					v1v2 = (n2.position - n.position);
					//v1v2 = util::normalize(v1v2);
					diff = (tmp - dist);
					n.position -= (v1v2 * diff) * .5f;
					n2.position += v1v2 * diff * .5f;
				}
			}
		}
	}

	void updatePT2() {
		register float diff, dist;
		register sf::Vector2f v1v2;
		const float tmp = 1;
		for (unsigned int i = 0; i < b.nodes.size(); ++i) {
			node& n = b.nodes[i];
			//n.position += n.velocity;
			//n.velocity *= .1f;
			/*
			for (unsigned int j = 0; j < n.neighboringVertices.size(); ++j) {
				node* n2 = n.neighboringVertices[j];
				dist = util::distance(n.position, n2->position);
				if (dist < tmp) {
					v1v2 = (n2->position - n.position);
					v1v2 = util::normalize(v1v2);
					diff = (tmp - dist);
					n.position -= v1v2 * diff * .5f;
					n2->position += v1v2 * diff * .5f;
				}
			}
			*/
			for (unsigned int j = 0; j < n.corners.size(); j++) {
				vector<node*> c = n.corners[j];
				float og = n.originalAngles[j];
				float angle = util::angleBetweenThreePoints(c[1]->position, c[0]->position, c[2]->position);
				//n.rotateAroundPoint(c[1]->position, -(og - angle));
				//n.rotateAroundPoint(c[2]->position, (og - angle));
;			}
		}
	}

	void render(sf::RenderTexture& RT) {
		// edges
		for (unsigned int i = 0; i < b.edges.size(); ++i) {
			register edge& E = b.edges[i];
			register sf::RectangleShape rect;
			float length = util::distance(E.n1->position.x, E.n2->position.x, E.n1->position.y, E.n2->position.y) * PIXELS_PER_UNIT;
			rect.setFillColor(sf::Color::White);
			rect.setSize(sf::Vector2f(LINE_THICKNESS, length));
			rect.setOrigin(sf::Vector2f(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2));
			register sf::Vector2f midpoint = E.n1->position + E.n2->position;
			midpoint.x /= 2;
			midpoint.y /= 2;
			midpoint.x *= PIXELS_PER_UNIT;
			midpoint.y *= PIXELS_PER_UNIT;
			float rotation = atan2(E.n1->position.y - E.n2->position.y, E.n1->position.x - E.n2->position.x);
			// converts radians to degrees
			rotation = rotation * 180 / 3.14159;
			rect.setPosition(midpoint);
			rect.setRotation(rotation + 90);

			RT.draw(rect);
		}



		// nodes
		for (unsigned int i = 0; i < b.nodes.size(); ++i) {
			node& n = b.nodes[i];
			register sf::RectangleShape nodeR = sf::RectangleShape(sf::Vector2f(NODE_DIAMETER, NODE_DIAMETER));
			sf::Vector2f pos = n.position;
			pos.x *= PIXELS_PER_UNIT;
			pos.y *= PIXELS_PER_UNIT;
			nodeR.setOrigin(nodeR.getGlobalBounds().width / 2, nodeR.getGlobalBounds().height / 2);
			nodeR.setPosition(pos);
			nodeR.setFillColor(sf::Color::White);
			nodeR.rotate(45); // node diamond (a rect rotated 45 degrees)
			RT.draw(nodeR);
		}

		debug::drawDebugPoints();
	}
}
