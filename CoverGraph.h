//
// Created by Tim on 20.03.2024.
//

#ifndef HIERARCHICALSYSTEM_COVERGRAPH_H
#define HIERARCHICALSYSTEM_COVERGRAPH_H

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "CoverEdge.h"
#include "Edge.h"
#include "Graph.h"
#include "Node.h"

// struct label{
//     int node;
//     std::vector<int> parameters;
//     label *parent;
//     int usedEdge;
//
//     label(int n, std::vector<int> parameters, label *p, int edge) : node(n), parameters(std::move(parameters)),
//     parent(p), usedEdge(edge) {}
//
//     void reset(int n, const std::vector<int>& params, label* p, int edge) {
//         node = n;
//         parameters = params;
//         parent = p;
//         usedEdge = edge;
//     }
// };
//
// struct lexCompare{
//     bool operator() (label* lhs, label* rhs) const {
//         for (int i = 0; i < (int)lhs->parameters.size(); ++i) {
//             if (lhs->parameters[i] < rhs->parameters[i])
//                 return true;
//             else if (lhs->parameters[i] > rhs->parameters[i])
//                 return false;
//         }
//
//         return lhs->node < rhs->node;
//     }
// };

/**
 * @class CoverGraph
 * @brief Hierarchical cover graph abstraction over the original graph for faster routing.
 *
 * Maintains cover nodes and edges, adjacency, and utilities for building the cover,
 * updating cluster information, and running MLS-based searches on the cover level.
 */
class CoverGraph {
   private:
    bool PRUNE_DOMINATED = false;  ///< Enable pruning of dominated edges during cover edge construction

    int clusterID = -1;                                               ///< Cluster ID this cover corresponds to
    int numCoverNodes = -1;                                           ///< Number of cover nodes
    int numCoverEdges = -1;                                           ///< Number of cover edges
    std::shared_ptr<const CoverGraph> precedingCoverGraph = nullptr;  ///< Previous level cover graph
    std::set<int> coverNodes;                                         ///< Set of node IDs that are cover nodes
    std::vector<CoverEdge> coverEdges;                                ///< List of cover edges
    // For outgoing and incoming edges, the first key is the node, the second key is the id of the original
    // outgoing/incoming edge
    std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>
        outgoingCoverEdges;  ///< Cover adjacency (outgoing)
    std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>
        incomingCoverEdges;                             ///< Cover adjacency (incoming)
    std::unordered_map<int, int> originalToCoverEdges;  ///< Map original edge -> cover edge

    const Graph& originalGraph;  ///< Reference to the underlying original graph

   public:
    // Constructor
    /** @brief Move constructor. */
    CoverGraph(CoverGraph&& other) noexcept;

    /** @brief Copy constructor. */
    CoverGraph(const CoverGraph& other);

    /** @brief Construct next-level cover from preceding cover. */
    CoverGraph(const std::shared_ptr<CoverGraph>& precedingCoverGraph);

    /** @brief Construct base-level cover over the original graph. */
    CoverGraph(const Graph& originalGraph);

    /** @brief Update cluster filter used by this cover graph. */
    void updateClusterID(int clusterOpt);

    /** @brief Build k-path cover for the current cluster. */
    void kPathCover();

    /** @brief Build the set of cover nodes. */
    void buildCoverNodes();

    /** @brief Build the set of cover edges and adjacency. */
    void buildCoverEdges();

    /** @brief Get cover edge by ID. */
    const CoverEdge& getCoverEdge(int i) const;

    /** @brief Get outgoing adjacency map for a cover node. */
    const std::unordered_map<int, std::unordered_set<int>>& getOutgoingEdges(int i) const;

    /** @brief Get incoming adjacency map for a cover node. */
    const std::unordered_map<int, std::unordered_set<int>>& getIncomingEdges(int i) const;

    /** @brief Access the underlying original graph. */
    const Graph& getOriginalGraph() const;

    /** @brief Get pair of start/end node IDs for a cover edge. */
    std::vector<int> getEdgeNodes(int i) const;

    /** @brief Get the list of original edges represented by a cover edge. */
    std::vector<int> getEdgeEdges(int i) const;

    /** @brief Link two partial paths' criteria across their hop counts. */
    std::vector<double> linkPaths(const std::vector<double>& currentCriteria, int numHops1,
                                  const std::vector<double>& suffixCriteria, int numHops2) const;

    /** @brief Check classic domination of two vectors. */
    bool checkDomination(const std::vector<double>& vector1, const std::vector<double>& vector2) const;

    /** @brief Forward MLS on k-path cover starting from a source edge. */
    std::map<int, std::set<label*, lexCompare>> tKpcMls(int source);

    /** @brief Backward MLS on k-path cover from a target edge. */
    std::map<int, std::set<label*, lexCompare>> backwardTKpcMLS(int target);

    /** @brief Number of cover edges. */
    int getNumEdges() const;

    /** @brief Number of cover nodes. */
    int getNumNodes() const;

    /** @brief Update t-set with a new label for t-discarding. */
    void updateTSet(std::set<std::vector<double>>& tSet, label* lab) const;

    /** @brief Perform t-discard dominance check against closed labels. */
    bool tDiscard(std::set<label*, lexCompare>& closedLabels, std::set<std::vector<double>>& tSet,
                  std::vector<double>& newVec);

    /** @brief Check if t-vector dominates v-vector. */
    bool checkTDomination(const std::vector<double>& t, const std::vector<double>& v) const;

    /** @brief Get set of cover node IDs. */
    std::set<int> getCoverNodes() const;

    /** @brief Whether node has any outgoing cover edges. */
    bool hasOutgoingEdges(int i) const;

    /** @brief Whether node has any incoming cover edges. */
    bool hasIncomingEdges(int i) const;

    /** @brief Get outgoing cover edges connected to an incoming edge. */
    std::unordered_set<int> getConnectedOutgoingEdges(int incomingEdge) const;

    /** @brief Get incoming cover edges connected to an outgoing edge. */
    std::unordered_set<int> getConnectedIncomingEdges(int outgoingEdge) const;

    /** @brief Get current cluster ID. */
    int getClusterID() const;

    /** @brief Set current cluster ID. */
    void setClusterID(int id);

    /** @brief Update dynamic info for cluster-dependent attributes. */
    void updateClusterInfo(const std::unordered_map<int, double>& densities,
                           const std::unordered_map<int, double>& speeds, const std::unordered_map<int, double>& fuels,
                           std::optional<int> clusterOpt);

    /** @brief Get pointer to preceding cover graph (previous level). */
    std::shared_ptr<const CoverGraph> getPrecedingCoverGraph() const;

    /** @brief Map original edge ID to the corresponding cover edge. */
    const CoverEdge& originalToCoverEdge(int originalEdge) const;

    /** @brief Get immutable list of all cover edges. */
    const std::vector<CoverEdge>& getCoverEdges() const;
};

#endif  // HIERARCHICALSYSTEM_COVERGRAPH_H
