#ifndef HIERARCHICALSYSTEM_GRAPH_H
#define HIERARCHICALSYSTEM_GRAPH_H


#include "Node.h"
#include "Edge.h"
#include <set>
#include <map>
#include <stack>
#include <iostream>
#include <utility>



struct label {
    int node;
    std::vector<double> parameters;
    label* pParent;
    Edge &usedEdge;

    label(int n, std::vector<double> parameters, label* p, Edge &edge) : node(n), parameters(std::move(parameters)), pParent(p), usedEdge(edge){}

//    label(int n, std::vector<double> parameters, label* p) : node(n), parameters(std::move(parameters)),
//                                                                        pParent(p){}

};

struct lexCompare {
    bool operator() (label* pLhs, label* pRhs) const {
        for (int i = 0; i < (int)pLhs->parameters.size(); ++i) {
            if (pLhs->parameters[i] < pRhs->parameters[i])
                return true;
            else if (pLhs->parameters[i] > pRhs->parameters[i])
                return false;
        }

        return pLhs->node < pRhs->node;
    }
};

class Graph {
private:
    std::vector<Node> nodes;
    std::vector<Edge> edges;

public:
    std::set<label*, lexCompare> mls(int sourceEdge, int goalEdge) {
        // int1 - node, int2 - out_edge
        std::map<int, std::map<int, std::set<label*, lexCompare>>> openLabels;
        std::map<int, std::map<int, std::set<label*, lexCompare>>> closedLabels;

        // Length, speed limit

        int startNode = edges[sourceEdge].getStartNode();
        int goalNode = edges[goalEdge].getStartNode();

        // Start label
        auto* sourceLabel = new label(startNode, std::vector<double>{edges[sourceEdge].getLength(), edges[sourceEdge].getSpeedLimit()}, nullptr, edges[sourceEdge]);
        // Specify queue
        std::set<label*, lexCompare> toExpand;
        toExpand.insert(sourceLabel);
        openLabels[startNode][sourceEdge].insert(sourceLabel);

        int iter = 0;
        int hitReached = 0;
        while (!toExpand.empty()) {
//            iter += 1;
//
//            if (iter % 10000 == 0) {
//                std::cout << toExpand.size() << std::endl;
//            }

            label* currentLabel = *toExpand.begin();
            toExpand.erase(toExpand.begin());
            int currentNode = currentLabel->node;
            std::vector<double> currentParameters = currentLabel->parameters;
            Edge& currentEdge = currentLabel->usedEdge;

            if (currentNode == goalNode && currentEdge.getID() == goalEdge) {
                hitReached += 1;
                std::cout << currentParameters[0] << " " << currentParameters[1] << std::endl;
            }

            openLabels[currentNode][currentEdge.getID()].erase(currentLabel);
            closedLabels[currentNode][currentEdge.getID()].insert(currentLabel);

            Node& nextNode = nodes[currentEdge.getEndNode()];
            for (int e : currentEdge.getOutgoingEdges()) {
                Edge& nextEdge = edges[e];

                std::vector<double> newParams(currentParameters);
                newParams[0] += nextEdge.getLength();
                newParams[1] += nextEdge.getSpeedLimit();

                // New edge created, compare against closed and open labels
                bool isDominated = false;
                std::set<label*, lexCompare> &closed = closedLabels[nextNode.getId()][nextEdge.getID()];
                for (auto l : closed) {
                    if (dominates(l->parameters, newParams)) {
                        isDominated = true;
                        break;
                    }
                }
                if (isDominated)
                    continue;

                std::stack<label*> toRemove;
                std::set<label*, lexCompare> &open = openLabels[nextNode.getId()][nextEdge.getID()];
                for (auto l : open) {
                    if (dominates(l->parameters, newParams)){
                        isDominated = true;
                        break;
                    }
                    else if (dominates(newParams, l->parameters)) {
                        toRemove.push(l);
                    }
                }
                if (isDominated)
                    continue;

                while (!toRemove.empty()) {
                    auto l = toRemove.top();
                    toRemove.pop();
                    open.erase(l);
                    toExpand.erase(l);
                    delete l;
                }

                auto* newLabel = new label(nextNode.getId(), newParams, currentLabel, nextEdge);
                toExpand.insert(newLabel);
                open.insert(newLabel);
            }
        }

//        for (const auto& p1 : closedLabels) {
//            for (const auto& p2 : p1.second) {
//                if (p1.first == goalNode && p2.first == goalEdge)
//                    continue;
//                for (label* l : p2.second) {
//                    delete l;
//                }
//            }
//        }

        return closedLabels[goalNode][goalEdge];
    }

    static bool dominates(std::vector<double>& v1, std::vector<double>& v2) {
        for (int i = 0; i < v1.size(); ++i) {
            if (v1[i] > v2[i] + 0.001) {
                return false;
            }
        }
        return true;
    }

    Graph(std::vector<Node> &nodes, std::vector<Edge> &edges) {
        this->nodes = std::move(nodes);
        this->edges = std::move(edges);
    }

    const std::vector<Node> &getNodes() const {
        return nodes;
    }

    const std::vector<Edge> &getEdges() const {
        return edges;
    }

    const Node &getNode(int id) const {
        return nodes[id];
    }

    const Edge &getEdge(int id) const {
        return edges[id];
    }

    void addNode(const Node& node) {
        nodes.push_back(node);
    }

    void addEdge(const Edge& edge) {
        edges.push_back(edge);
    }

    int getNumNodes() const {
        return nodes.size();
    }

    int getNumEdges() const {
        return edges.size();
    }


};


#endif //HIERARCHICALSYSTEM_GRAPH_H
