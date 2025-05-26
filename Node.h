//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_NODE_H
#define HIERARCHICALSYSTEM_NODE_H

#include <utility>
#include <vector>
#include <utility>
#include <unordered_set>

class Node {
private:
    int id;
    int clusterBitmask;
    std::pair<double, double> coordinates;
    std::unordered_set<int> incomingEdges;
    std::unordered_set<int> outgoingEdges;
public:
    Node(int id, std::pair<double, double> coordinates, std::unordered_set<int>& incomingEdges,
         std::unordered_set<int>& outgoingEdges, int cluster) {
        this->id = id;
        this->coordinates = std::move(coordinates);
        this->incomingEdges = std::move(incomingEdges);
        this->outgoingEdges = std::move(outgoingEdges);
        this->clusterBitmask = cluster;
    }

    int getId() const {
        return id;
    }

    std::pair<double, double> getCoordinates() const {
        return coordinates;
    }

    const std::unordered_set<int> &getIncomingEdges() const {
        return incomingEdges;
    }

    const std::unordered_set<int> &getOutgoingEdges() const {
        return outgoingEdges;
    }

    void addIncomingEdge(int edgeId) {
        incomingEdges.insert(edgeId);
    }

    void addOutgoingEdge(int edgeId) {
        outgoingEdges.insert(edgeId);
    }

    bool inCluster(int clusterID) const {
        return clusterBitmask & (1 << clusterID);
    }

    bool isBorderNode() const {
        // Check if more than one bit is set in nodeMask
        return (clusterBitmask & (clusterBitmask - 1)) != 0;
    }

    int getClusterBitmask() const {
        return clusterBitmask;
    }

};


#endif //HIERARCHICALSYSTEM_NODE_H
