//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_COVEREDGE_H
#define HIERARCHICALSYSTEM_COVEREDGE_H

#include <deque>
#include <functional>

#include "Edge.h"

class CoverEdge {
   private:
    int id;
    int clusterBitmask;
    double length;
    double speedLimit;
    double numLanes;
    double maxThroughput;
    double dangerFactor;

    std::vector<int> insideNodes;
    std::vector<int> insideEdges;
    std::pair<int, int> compositeCoverEdges;

   public:
    // Constructor for combining two cover edges
    explicit CoverEdge(const CoverEdge &firstEdge, const CoverEdge &secondEdge);

    explicit CoverEdge(const Edge &originalEdge);

    CoverEdge(const CoverEdge &coverEdge);

    CoverEdge(int id, int clusterBitmask, const std::vector<int> &insideEdges);

    const std::vector<int> &getInsideNodes() const;

    int getStartNode() const;

    int getEndNode() const;

    const std::vector<int> &getInsideEdges() const;

    int getStartEdge() const;

    int getEndEdge() const;

    std::pair<int, int> getCompositeCoverEdges() const;

    double getLength() const;

    double getSpeedLimit() const;

    double getDangerFactor() const;

    double getNumLanes() const;

    double getMaxThroughput() const;

    // std::vector<double> getPlanningParameters() const;

    std::vector<double> getBuildingParameters() const;

    int getId() const;

    void setId(int id);

    int getClusterBitmask() const;

    void setClusterBitmask(int id);

    bool inCluster(int clusterID) const;

    int getNumHops() const;
};
#endif  // HIERARCHICALSYSTEM_COVEREDGE_H
