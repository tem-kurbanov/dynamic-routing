#ifndef HIERARCHICALSYSTEM_EDGE_H
#define HIERARCHICALSYSTEM_EDGE_H

#include <set>
#include <unordered_set>
#include <vector>

class Edge {
   private:
    int id;         ///< Unique identifier of the edge
    int startNode;  ///< ID of the start node
    int endNode;    ///< ID of the end node

    int roadType;         ///< Encoded road type/classification
    double length;        ///< Edge length (meters)
    int numLanes;         ///< Number of lanes
    double speedLimit;    ///< Speed limit (m/s or km/h depending on convention)
    double tlsCoeff;      ///< Traffic light slowdown coefficient
    double dangerFactor;  ///< Safety risk factor for the edge

    int clusterBitmask;  ///< Cluster membership bitmask

    std::unordered_set<int> incomingEdges;  ///< IDs of predecessor edges
    std::unordered_set<int> outgoingEdges;  ///< IDs of successor edges

   public:
    /**
     * @brief Construct an edge with attributes and adjacency.
     */
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

    /** @brief Default-construct an invalid edge placeholder. */
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

    /** @brief Get edge identifier. */
    int getID() const {
        return id;
    }

    /** @brief Get start node ID. */
    int getStartNode() const {
        return startNode;
    }

    /** @brief Get end node ID. */
    int getEndNode() const {
        return endNode;
    }

    /** @brief Get edge length. */
    double getLength() const {
        return length;
    }

    /** @brief Get number of lanes. */
    int getNumLanes() const {
        return numLanes;
    }

    /** @brief Get speed limit. */
    double getSpeedLimit() const {
        return speedLimit;
    }

    /** @brief Get predecessor edges. */
    std::unordered_set<int> getIncomingEdges() const {
        return incomingEdges;
    }

    /** @brief Get successor edges. */
    std::unordered_set<int> getOutgoingEdges() const {
        return outgoingEdges;
    }

    /** @brief Check if the edge belongs to a given cluster. */
    bool inCluster(int clusterID) const {
        return clusterBitmask & (1 << clusterID);
    }

    /** @brief Determine if the edge spans cluster border. */
    bool isBorderEdge() const {
        // Check if more than one bit is set in nodeMask
        return (clusterBitmask & (clusterBitmask - 1)) != 0;
    }

    /** @brief Get the raw cluster bitmask. */
    int getClusterBitmask() const {
        return clusterBitmask;
    }

    /** @brief Get traffic light slowdown coefficient. */
    double getTlsCoeff() const {
        return tlsCoeff;
    }

    /** @brief Get encoded road type. */
    int getRoadType() const {
        return roadType;
    }

    /** @brief Get safety risk factor. */
    double getDangerFactor() const {
        return dangerFactor;
    }

    /** @brief Maximum density estimate based on lanes and vehicle length. */
    double getMaxDensity() const {
        return 1000 * numLanes / 7.5;
    }
};

#endif  // HIERARCHICALSYSTEM_EDGE_H
