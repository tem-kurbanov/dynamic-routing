#ifndef HIERARCHICALSYSTEM_GRAPH_H
#define HIERARCHICALSYSTEM_GRAPH_H

#include <iostream>
#include <map>
#include <set>
#include <stack>
#include <utility>

#include "Edge.h"
#include "Node.h"

/**
 * @struct label
 * @brief Label used in multi-criteria label-setting search on the original graph.
 *
 * Holds current node, accumulated parameter vector and back-pointer to parent label,
 * and the edge used to reach the node.
 */
struct label {
    int node;                        ///< Current node ID
    std::vector<double> parameters;  ///< Accumulated criteria vector
    label* pParent;                  ///< Parent label pointer (ownership external)
    Edge& usedEdge;                  ///< Edge used to reach current node

    /** @brief Construct a search label. */
    label(int n, std::vector<double> parameters, label* p, Edge& edge)
        : node(n), parameters(std::move(parameters)), pParent(p), usedEdge(edge) {
    }

    //    label(int n, std::vector<double> parameters, label* p) : node(n), parameters(std::move(parameters)),
    //                                                                        pParent(p){}
};

/**
 * @struct lexCompare
 * @brief Lexicographic comparator for label pointers by their parameter vectors.
 */
struct lexCompare {
    bool operator()(label* pLhs, label* pRhs) const {
        for (int i = 0; i < (int) pLhs->parameters.size(); ++i) {
            if (pLhs->parameters[i] < pRhs->parameters[i])
                return true;
            else if (pLhs->parameters[i] > pRhs->parameters[i])
                return false;
        }

        return pLhs->node < pRhs->node;
    }
};

/**
 * @class Graph
 * @brief Original road network graph with nodes and edges.
 */
class Graph {
   private:
    std::vector<Node> nodes;  ///< List of graph nodes
    std::vector<Edge> edges;  ///< List of graph edges

   public:
    /**
     * @brief Multi-criteria label-setting (MLS) search between two edges.
     * @param sourceEdge ID of source edge
     * @param goalEdge ID of goal edge
     * @return Set of non-dominated labels at the goal state
     */
    std::set<label*, lexCompare> mls(int sourceEdge, int goalEdge) {
        // int1 - node, int2 - out_edge
        std::map<int, std::map<int, std::set<label*, lexCompare>>> openLabels;
        std::map<int, std::map<int, std::set<label*, lexCompare>>> closedLabels;

        // Length, speed limit

        int startNode = edges[sourceEdge].getStartNode();
        int goalNode = edges[goalEdge].getStartNode();

        // Start label
        auto* sourceLabel =
            new label(startNode, std::vector<double>{edges[sourceEdge].getLength(), edges[sourceEdge].getSpeedLimit()},
                      nullptr, edges[sourceEdge]);
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
                std::set<label*, lexCompare>& closed = closedLabels[nextNode.getId()][nextEdge.getID()];
                for (auto l : closed) {
                    if (dominates(l->parameters, newParams)) {
                        isDominated = true;
                        break;
                    }
                }
                if (isDominated)
                    continue;

                std::stack<label*> toRemove;
                std::set<label*, lexCompare>& open = openLabels[nextNode.getId()][nextEdge.getID()];
                for (auto l : open) {
                    if (dominates(l->parameters, newParams)) {
                        isDominated = true;
                        break;
                    } else if (dominates(newParams, l->parameters)) {
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

    /**
     * @brief Check Pareto domination (v1 dominates v2).
     */
    static bool dominates(std::vector<double>& v1, std::vector<double>& v2) {
        for (int i = 0; i < v1.size(); ++i) {
            if (v1[i] > v2[i] + 0.001) {
                return false;
            }
        }
        return true;
    }

    /** @brief Construct a graph from nodes and edges. */
    Graph(std::vector<Node>& nodes, std::vector<Edge>& edges) {
        this->nodes = std::move(nodes);
        this->edges = std::move(edges);
    }

    /** @brief Get all nodes. */
    const std::vector<Node>& getNodes() const {
        return nodes;
    }

    /** @brief Get all edges. */
    const std::vector<Edge>& getEdges() const {
        return edges;
    }

    /** @brief Get node by ID. */
    const Node& getNode(int id) const {
        return nodes[id];
    }

    /** @brief Get edge by ID. */
    const Edge& getEdge(int id) const {
        return edges[id];
    }

    /** @brief Append a node to the graph. */
    void addNode(const Node& node) {
        nodes.push_back(node);
    }

    /** @brief Append an edge to the graph. */
    void addEdge(const Edge& edge) {
        edges.push_back(edge);
    }

    /** @brief Number of nodes. */
    int getNumNodes() const {
        return nodes.size();
    }

    /** @brief Number of edges. */
    int getNumEdges() const {
        return edges.size();
    }
};

#endif  // HIERARCHICALSYSTEM_GRAPH_H
