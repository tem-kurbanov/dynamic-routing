//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_COVEREDGE_H
#define HIERARCHICALSYSTEM_COVEREDGE_H

#include <deque>
#include <functional>

#include "Edge.h"

/**
 * @class CoverEdge
 * @brief Edge in a hierarchical cover graph representing one or several original edges.
 */
class CoverEdge {
   private:
    int id;                ///< Unique identifier of the cover edge
    int clusterBitmask;    ///< Cluster membership bitmask
    double length;         ///< Aggregated length of the covered subpath
    double speedLimit;     ///< Representative speed limit
    double numLanes;       ///< Effective number of lanes
    double maxThroughput;  ///< Estimated maximum throughput
    double dangerFactor;   ///< Aggregated safety risk factor

    std::vector<int> insideNodes;             ///< IDs of original nodes inside this cover edge
    std::vector<int> insideEdges;             ///< IDs of original edges inside this cover edge
    std::pair<int, int> compositeCoverEdges;  ///< IDs of two cover edges combined to form this one

   public:
    /** @brief Construct a cover edge by combining two cover edges. */
    explicit CoverEdge(const CoverEdge &firstEdge, const CoverEdge &secondEdge);

    /** @brief Construct a cover edge from an original edge. */
    explicit CoverEdge(const Edge &originalEdge);

    /** @brief Copy-construct a cover edge. */
    CoverEdge(const CoverEdge &coverEdge);

    /** @brief Construct with explicit IDs and composition. */
    CoverEdge(int id, int clusterBitmask, const std::vector<int> &insideEdges);

    /** @brief Get original nodes inside the cover edge. */
    const std::vector<int> &getInsideNodes() const;

    /** @brief Get start node ID of the cover edge. */
    int getStartNode() const;

    /** @brief Get end node ID of the cover edge. */
    int getEndNode() const;

    /** @brief Get original edge IDs inside the cover edge. */
    const std::vector<int> &getInsideEdges() const;

    /** @brief Get start original edge ID. */
    int getStartEdge() const;

    /** @brief Get end original edge ID. */
    int getEndEdge() const;

    /** @brief Get pair of composing cover edges if applicable. */
    std::pair<int, int> getCompositeCoverEdges() const;

    /** @brief Get aggregated length. */
    double getLength() const;

    /** @brief Get representative speed limit. */
    double getSpeedLimit() const;

    /** @brief Get aggregated danger factor. */
    double getDangerFactor() const;

    /** @brief Get effective number of lanes. */
    double getNumLanes() const;

    /** @brief Get estimated maximum throughput. */
    double getMaxThroughput() const;

    // std::vector<double> getPlanningParameters() const;

    /** @brief Get static parameter vector used for building covers. */
    std::vector<double> getBuildingParameters() const;

    /** @brief Get cover edge identifier. */
    int getId() const;

    /** @brief Set cover edge identifier. */
    void setId(int id);

    /** @brief Get cluster bitmask. */
    int getClusterBitmask() const;

    /** @brief Set cluster bitmask. */
    void setClusterBitmask(int id);

    /** @brief Check if the cover edge belongs to a given cluster. */
    bool inCluster(int clusterID) const;

    /** @brief Number of original edges represented by this cover edge. */
    int getNumHops() const;
};
#endif  // HIERARCHICALSYSTEM_COVEREDGE_H
