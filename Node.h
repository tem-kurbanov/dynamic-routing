//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_NODE_H
#define HIERARCHICALSYSTEM_NODE_H

#include <unordered_set>
#include <utility>
#include <vector>

/**
 * @class Node
 * @brief Vertex of the original road network graph.
 *
 * Represents a road network node with geographic coordinates and incident
 * incoming/outgoing edge identifiers. Nodes may belong to multiple clusters,
 * encoded as a bitmask.
 */
class Node {
   private:
    int id;                                 ///< Unique identifier of the node
    int clusterBitmask;                     ///< Bitmask of clusters this node belongs to
    std::pair<double, double> coordinates;  ///< Geographic or projected coordinates
    std::unordered_set<int> incomingEdges;  ///< IDs of incoming edges terminating at this node
    std::unordered_set<int> outgoingEdges;  ///< IDs of outgoing edges originating from this node
   public:
    /**
     * @brief Construct a Node with its geometry and adjacency.
     * @param id Unique node identifier
     * @param coordinates Node coordinates
     * @param incomingEdges Set of incoming edge IDs
     * @param outgoingEdges Set of outgoing edge IDs
     * @param cluster Cluster membership bitmask
     */
    Node(int id, std::pair<double, double> coordinates, std::unordered_set<int>& incomingEdges,
         std::unordered_set<int>& outgoingEdges, int cluster) {
        this->id = id;
        this->coordinates = std::move(coordinates);
        this->incomingEdges = std::move(incomingEdges);
        this->outgoingEdges = std::move(outgoingEdges);
        this->clusterBitmask = cluster;
    }

    /** @brief Get the node identifier. */
    int getId() const {
        return id;
    }

    /** @brief Get node coordinates. */
    std::pair<double, double> getCoordinates() const {
        return coordinates;
    }

    /** @brief Get IDs of edges incoming to the node. */
    const std::unordered_set<int>& getIncomingEdges() const {
        return incomingEdges;
    }

    /** @brief Get IDs of edges outgoing from the node. */
    const std::unordered_set<int>& getOutgoingEdges() const {
        return outgoingEdges;
    }

    /** @brief Add an incoming edge ID. */
    void addIncomingEdge(int edgeId) {
        incomingEdges.insert(edgeId);
    }

    /** @brief Add an outgoing edge ID. */
    void addOutgoingEdge(int edgeId) {
        outgoingEdges.insert(edgeId);
    }

    /**
     * @brief Check if the node belongs to a given cluster.
     * @param clusterID Cluster identifier (bit index)
     */
    bool inCluster(int clusterID) const {
        return clusterBitmask & (1 << clusterID);
    }

    /**
     * @brief Determine whether the node is a border node.
     * @return True if the node belongs to more than one cluster.
     */
    bool isBorderNode() const {
        // Check if more than one bit is set in nodeMask
        return (clusterBitmask & (clusterBitmask - 1)) != 0;
    }

    /** @brief Get the raw cluster bitmask. */
    int getClusterBitmask() const {
        return clusterBitmask;
    }
};

#endif  // HIERARCHICALSYSTEM_NODE_H
