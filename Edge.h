#ifndef HIERARCHICALSYSTEM_EDGE_H
#define HIERARCHICALSYSTEM_EDGE_H

#include <set>
#include <unordered_set>
#include <vector>

class Edge {
   private:
    int id;
    int startNode;
    int endNode;

    int roadType;
    double length;
    int numLanes;
    double speedLimit;
    double tlsCoeff;
    double dangerFactor;

    int clusterBitmask;

    std::unordered_set<int> incomingEdges;
    std::unordered_set<int> outgoingEdges;

   public:
    Edge(int id, int startNode, int endNode, int roadType, double length, double speedLimit, int numLanes,
         double tlsCoeff, double dangerFactor, std::unordered_set<int> &incomingEdges,
         std::unordered_set<int> &outgoingEdges, int cluster) {
        this->id = id;
        this->startNode = startNode;
        this->endNode = endNode;
        this->roadType = roadType;
        this->length = length;
        this->numLanes = numLanes;
        this->speedLimit = speedLimit;
        this->tlsCoeff = tlsCoeff;
        this->dangerFactor = dangerFactor;
        this->incomingEdges = std::move(incomingEdges);
        this->outgoingEdges = std::move(outgoingEdges);
        this->clusterBitmask = cluster;
    }

    Edge() {
        this->id = -1;
        this->startNode = -1;
        this->endNode = -1;
        this->length = -1;
        this->numLanes = -1;
        this->speedLimit = -1;
        this->tlsCoeff = -1;
        this->dangerFactor = -1;
        this->incomingEdges = std::unordered_set<int>();
        this->outgoingEdges = std::unordered_set<int>();
        this->clusterBitmask = 0;
    }

    int getID() const {
        return id;
    }

    int getStartNode() const {
        return startNode;
    }

    int getEndNode() const {
        return endNode;
    }

    double getLength() const {
        return length;
    }

    int getNumLanes() const {
        return numLanes;
    }

    double getSpeedLimit() const {
        return speedLimit;
    }

    std::unordered_set<int> getIncomingEdges() const {
        return incomingEdges;
    }

    std::unordered_set<int> getOutgoingEdges() const {
        return outgoingEdges;
    }

    bool inCluster(int clusterID) const {
        return clusterBitmask & (1 << clusterID);
    }

    bool isBorderEdge() const {
        // Check if more than one bit is set in nodeMask
        return (clusterBitmask & (clusterBitmask - 1)) != 0;
    }

    int getClusterBitmask() const {
        return clusterBitmask;
    }

    double getTlsCoeff() const {
        return tlsCoeff;
    }

    int getRoadType() const {
        return roadType;
    }

    double getDangerFactor() const {
        return dangerFactor;
    }

    double getMaxDensity() const {
        return 1000 * numLanes / 7.5;
    }
};

#endif  // HIERARCHICALSYSTEM_EDGE_H
