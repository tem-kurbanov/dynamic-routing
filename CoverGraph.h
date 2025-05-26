//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_COVERGRAPH_H
#define HIERARCHICALSYSTEM_COVERGRAPH_H

#include "Node.h"
#include "Edge.h"
#include "CoverEdge.h"
#include "Graph.h"

#include <set>
#include <map>
#include <vector>
#include <deque>
#include <memory>
#include <optional>

//struct label{
//    int node;
//    std::vector<int> parameters;
//    label *parent;
//    int usedEdge;
//
//    label(int n, std::vector<int> parameters, label *p, int edge) : node(n), parameters(std::move(parameters)), parent(p), usedEdge(edge) {}
//
//    void reset(int n, const std::vector<int>& params, label* p, int edge) {
//        node = n;
//        parameters = params;
//        parent = p;
//        usedEdge = edge;
//    }
//};
//
//struct lexCompare{
//    bool operator() (label* lhs, label* rhs) const {
//        for (int i = 0; i < (int)lhs->parameters.size(); ++i) {
//            if (lhs->parameters[i] < rhs->parameters[i])
//                return true;
//            else if (lhs->parameters[i] > rhs->parameters[i])
//                return false;
//        }
//
//        return lhs->node < rhs->node;
//    }
//};

class CoverGraph {
private:
    bool PRUNE_DOMINATED = false;

    int clusterID = -1;
    int numCoverNodes = -1;
    int numCoverEdges = -1;
    std::shared_ptr<const CoverGraph> precedingCoverGraph = nullptr;
    std::set<int> coverNodes;
    std::vector<CoverEdge> coverEdges;
    // For outgoing and incoming edges, the first key is the node, the second key is the id of the original outgoing/incoming edge
    std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> outgoingCoverEdges;
    std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> incomingCoverEdges;
    std::unordered_map<int, int> originalToCoverEdges;

    const Graph& originalGraph;

public:
    // Constructor
    CoverGraph(CoverGraph&& other) noexcept;

    CoverGraph(const CoverGraph& other);

    CoverGraph(const std::shared_ptr<CoverGraph>& precedingCoverGraph);

    CoverGraph(const Graph& originalGraph);

    void updateClusterID(int clusterOpt);

    void kPathCover();

    void buildCoverNodes();

    void buildCoverEdges();

    const CoverEdge& getCoverEdge(int i) const;

    const std::unordered_map<int, std::unordered_set<int>>& getOutgoingEdges(int i) const;

    const std::unordered_map<int, std::unordered_set<int>>& getIncomingEdges(int i) const;

    const Graph& getOriginalGraph() const;

    std::vector<int> getEdgeNodes(int i) const;

    std::vector<int> getEdgeEdges(int i) const;

    std::vector<double> linkPaths(const std::vector<double>& currentCriteria, int numHops1, const std::vector<double>& suffixCriteria, int numHops2) const;

    bool checkDomination(const std::vector<double> &vector1, const std::vector<double> &vector2) const;

    std::map<int, std::set<label*, lexCompare>> tKpcMls(int source);

    std::map<int, std::set<label *, lexCompare>> backwardTKpcMLS(int target);

    int getNumEdges() const;

    int getNumNodes() const;

    void updateTSet(std::set<std::vector<double>> &tSet, label *lab) const ;

    bool tDiscard(std::set<label *, lexCompare> &closedLabels, std::set<std::vector<double>> &tSet, std::vector<double> &newVec);

    bool checkTDomination(const std::vector<double> &t, const std::vector<double> &v) const ;

    std::set<int> getCoverNodes() const;

    bool hasOutgoingEdges(int i) const;

    bool hasIncomingEdges(int i) const;

    std::unordered_set<int> getConnectedOutgoingEdges(int incomingEdge) const;

    std::unordered_set<int> getConnectedIncomingEdges(int outgoingEdge) const;

    int getClusterID() const;

    void setClusterID(int id);

    void updateClusterInfo(const std::unordered_map<int, double>& densities, const std::unordered_map<int, double>& speeds, const std::unordered_map<int, double>& fuels, std::optional<int> clusterOpt);

    std::shared_ptr<const CoverGraph> getPrecedingCoverGraph() const;

    const CoverEdge& originalToCoverEdge(int originalEdge) const;

    const std::vector<CoverEdge>& getCoverEdges() const;
};



#endif //HIERARCHICALSYSTEM_COVERGRAPH_H
