#ifndef HIERARCHICALSYSTEM_ROUTER_H
#define HIERARCHICALSYSTEM_ROUTER_H

#include <mpi.h>

#include <string>
#include <tuple>
#include <utility>

#include "CoverGraph.h"
#include "FuelEstimator.h"
#include "Graph.h"
#include "SpeedEstimator.h"
#include "TrafficEstimator.h"
#include "serializations.h"

/**
 * @struct hierLabel
 * @brief Label structure for Hierarchical MLS
 *
 * Label structure for the Hierarchical MLS algorithm used both in forward and
 * backward searches
 */
struct hierLabel {
    /// ID of the node the label refers to
    int node;
    /// Parameters of the path represented by the label
    std::vector<double> parameters;
    /// Parent label in the search tree
    std::shared_ptr<hierLabel> pParent;
    /// Cover edge used to reach the node
    const std::shared_ptr<CoverEdge> usedEdge;
    /// Number of hops to reach the node in the original graph
    int numHops;

    /**
     * @brief Standard constructor for a new label
     * @param n Node ID
     * @param parameters Parameters of the path represented by the label
     * @param p Parent label in the search tree
     * @param edge Cover edge used to reach the node
     * @param nh Number of hops to reach the node in the original graph
     *
     * Standard constructor for a new label used in computations in own cluster
     */
    hierLabel(int n, std::vector<double> parameters, std::shared_ptr<hierLabel> p,
              const std::shared_ptr<CoverEdge> edge, int nh)
        : node(n), parameters(std::move(parameters)), pParent(std::move(p)), usedEdge(std::move(edge)), numHops(nh) {
    }

    /**
     * @brief Constructor of label received from another cluster
     * @param n Node ID
     * @param parameters Parameters of the path represented by the label
     * @param edge Cover edge used to reach the node
     * @param nh Number of hops to reach the node in the original graph
     *
     * Constructor used to create a label received from another cluster to start
     * Hierarchical MLS search in own cluster
     */
    hierLabel(int n, std::vector<double> parameters, const std::shared_ptr<CoverEdge> edge, int nh)
        : node(n), parameters(std::move(parameters)), pParent(nullptr), usedEdge(std::move(edge)), numHops(nh) {
    }
};

/**
 * @struct hierLexCompare
 *
 * Structure that performs lexicographic comparison of labels for label-setting
 * path search
 */
struct hierLexCompare {
    /**
     * @brief Lexicographic comparison of two labels (raw pointers)
     * @param pLhs "left" label
     * @param pRhs "right" label
     * @return True if the left label is lexicographically smaller than the right
     * label
     */
    bool operator()(hierLabel *pLhs, hierLabel *pRhs) const {
        for (int i = 0; i < (int) pLhs->parameters.size(); ++i) {
            if (pLhs->parameters[i] < pRhs->parameters[i])
                return true;
            else if (pLhs->parameters[i] > pRhs->parameters[i])
                return false;
        }

        return pLhs->node < pRhs->node;
    }

    /**
     * @brief Lexicographic comparison of two labels (shared pointers)
     * @param pLhs "left" label
     * @param pRhs "right" label
     * @return True if the left label is lexicographically smaller than the right
     * label
     */
    bool operator()(const std::shared_ptr<hierLabel> &pLhs, const std::shared_ptr<hierLabel> &pRhs) const {
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
 * @class Router
 * @brief Router class that handles dynamic routing in its own cluster
 *
 * Main class for dynamic routing in the system. It is responsible for proactive
 * and reactive routing of vehicles. Every instance of the class is a separate
 * agent with own MPI thread and assigned area of the road network.
 */
class Router {
   private:
    /// Total number of Rotuer agents in the system
    int NUM_AGENTS = -1;
    /// Number of parameters used in the routing algorithm
    int NUM_PARAMETERS = 4;
    /// Traffic estimator used to predict traffic density
    TrafficEstimator &mTrafficEstimator;
    /// Speed estimator used to predict traffic speed
    SpeedEstimator &mSpeedEstimator;
    /// Fuel estimator used to predict fuel consumption
    FuelEstimator &mFuelEstimator;
    /// ID of the cluster the Router is assigned to
    int mClusterID = -1;
    /// Map of bitmasks to cluster IDs
    std::unordered_map<int, int> mBitmaskToCluster;
    /// Simulation start time
    int simulationStart = 0;

    /// Map of vehicle IDs to their routes
    std::unordered_map<int, std::vector<int>> mVehicleRoutes;
    /// Map of vehicle IDs to their expected arrival times on the route edges
    /// (maps to mVehicleRoutes)
    std::unordered_map<int, std::vector<double>> mVehicleTimes;
    /// Map of vehicle IDs to their current positions on the route (maps to
    /// mVehicleRoutes)
    std::unordered_map<int, int> mVehiclePositions;  // in index on the route
    /// Map of numbers of vehicles currently on the edges of the cluster
    std::unordered_map<int, std::unordered_set<int>> mCurrentEdgePopulation;
    /// Map of vehicle IDs to their planning priorities
    std::unordered_map<int, std::vector<double>> mVehiclePriorities;

    /// Map of edge IDs to their flow vectors
    std::unordered_map<int, std::vector<std::pair<int, int>>> mEdgeFlows;
    /// Map of edge IDs to their densities (inner vector ids correspond to
    /// intervals, 0 being the current one)
    std::unordered_map<int, std::vector<double>> mEdgeDensities;
    /// Map of edge IDs to their speeds (inner vector ids correspond to intervals,
    /// 0 being the current one)
    std::unordered_map<int, std::vector<double>> mEdgeSpeeds;
    /// Map of edge IDs to their fuel consumptions (inner vector ids correspond to
    /// intervals, 0 being the current one)
    std::unordered_map<int, std::vector<double>> mEdgeFuels;

    // Internal hyperparameters
    /// Congestion threshold when reactive rerouting is triggered
    double mCongestionThreshold = 0.8;
    /// Current interval of the simulation
    int mCurrentInterval = 0;
    /// Number of intervals that have passed since the simulation start
    int mPassedIntervals = 0;
    /// Current time of the simulation
    double mCurrentTime = 0;
    /// Length of the simulation interval
    double mIntervalLength = 60;
    /// Portion of vehicles to be rerouted when congested edge is detected
    double mReroutePortion = 1.0;

    /// Hierarchical over graphs used for routing
    std::vector<std::shared_ptr<CoverGraph>> mCoverGraphs;
    /// Original graph of the road network
    Graph &mOriginalGraph;
    /// Top level of the hierarchical cover graph
    int mTopLevel;
    /// Level limit for the search
    int mLevelLimit;
    /// Map of node IDs to their top levels in the hierarchical cover graph
    std::vector<int> mNodeLevels;

    /// Enable/disable logging
    bool mEnableLogging = true;

    /// Epsilon pruning parameter for MLS search
    double epsilon = 1.0;

    void logMessage(const std::string &message) const;
    void logError(const std::string &error) const;

    // Management functions
    // ============================================================================================
    /** @brief Build initial cover graphs at the start of operation*/
    std::vector<std::shared_ptr<CoverGraph>> buildInitialCovers(int k);

    /** @brief Prepare for new interval and forget predictions from the old one */
    void nextInterval();

    // Predictions
    // =====================================================================================================

    /** @brief Predict dynamic parameters (density, traversal time, fuel
     * consumption) for the given cover edge at the given time */
    std::tuple<double, double, double> predictDynamicParameters(const CoverEdge &edge, int enterTime);

    /** @brief Predict dynamic parameters (density, speed, fuel consumption) for
     * the given edge at the given time */
    std::tuple<double, double, double> predictDynamicParameters(const Edge &edge, int enterTime);

    // Utility for routing
    // =============================================================================================

    /** @brief Selects the best Pareto route from the Pareto front using
     * R-method*/
    std::vector<double> selectBestRoute(std::vector<std::vector<double>> &candidates,
                                        const std::vector<double> &priorities);

    std::shared_ptr<hierLabel> selectFastestRoute(
        const std::set<std::shared_ptr<hierLabel>, hierLexCompare> &candidates, const std::vector<double> &priorities);

    /** @brief Build path vector of edges from the planning label */
    std::shared_ptr<hierLabel> buildForwardPath(const std::shared_ptr<hierLabel> &originalLabel,
                                                std::vector<int> &path);

    /** @brief Record provided route into inner memory and update dynamic
     * parameters */
    void recordRoute(int vehID, const std::vector<int> &route, std::vector<double> times,
                     const std::vector<double> &priorities, int currentTime);

    // Planning functions
    // ==============================================================================================

    /** @brief Perform completed hierarchical search for the given start and goal
     * edges (both belong to the current cluster) */
    std::set<std::shared_ptr<hierLabel>, hierLexCompare> completeHierarchicalSearch(
        int startEdge, int goalEdge,
        std::optional<
            std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
            openLabels = {});

    /** @brief Perform hierarchical search to edges of the area since the goal
     * edge is not in the current cluster */
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
    distributedHierarchicalSearch(
        int startEdge, int goalEdge, std::unordered_set<int> &extendableClusters,
        std::optional<
            std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
            openLabels = {});

    /** @brief Backward Hierarchical BFS search to find all paths connecting goal
     * edge to the top cover graph */
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
    backwardHierarchicalSearch(int goalEdge);

    // Utility for planning
    // ============================================================================================

    std::unordered_set<int> findExtendableClusters(
        int startEdge, int goalEdge, std::vector<int> &clusterSequence,
        std::optional<
            std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
            openLabels = {}) const;

    /** @brief Expand the label via the given edge */
    std::shared_ptr<hierLabel> expandLabel(const std::shared_ptr<hierLabel> &currentLabel,
                                           const std::shared_ptr<CoverEdge> &nextEdge);

    /** @brief Update current tSet with the newly created label */
    void updateTSet(std::set<std::vector<double>> &rTSet, const std::shared_ptr<hierLabel> &pLab);

    /** @brief Check if the new label is dominated by the current closed set using
     * t-discarding*/
    bool tDiscard(std::set<std::vector<double>> &rTSet, std::vector<double> &rNewVec) const;

    /** @brief Classic domination check */
    bool checkDomination(const std::vector<double> &rVector1, const std::vector<double> &rVector2) const;

    /** @brief Link two subpaths in terms of planning criteria */
    std::vector<double> linkPaths(const std::vector<double> &rCurrentCriteria, int numOriginalEdges1,
                                  const std::vector<double> &rSuffixCriteria, int numOriginalEdges2);

    /** @brief Prepare dynamic planning parameters (distance, traversal time, fuel
     * consumption, average density) for the given cover edge at the given time */
    std::vector<double> prepareDynamicParameters(const CoverEdge &edge, double enterTime);

    /** @brief Compute average dynamic planning parameters (distance, time, fuel
     * consumption, average density) for the given cover edge at the given time
     * based on recorded estimations*/
    std::vector<double> computeDynamicParameters(const CoverEdge &edge, double enterTime);

    /** @brief Extract recorded dynamic planning parameters (distance, time, fuel
     * consumption, average density) for the given edge at the given time */
    std::vector<double> extractDynamicParameters(const Edge &edge, double enterTime);

    /** @brief Compute which interval the given time belongs to */
    int timeToInterval(double time);

    /** @brief Compute which interval the given time belongs to */
    int timeToInterval(int time);

    // Reactive rerouting
    // ==============================================================================================

    /** @brief Use predicted densitied to identify upcoming congestions */
    std::unordered_set<int> predictCongestion(const std::unordered_map<int, double> &predictedDensities);

    /** @brief For every congested edges, select vehicles that would traverse it
     * and should be rerouted */
    std::unordered_set<int> selectForRerouting(const std::unordered_set<int> &congestedEdges);

    // Communication functions
    // =========================================================================================

    // Continue route
    // --------------------------------------------------------------------------------------------------

    /** @brief Send message to continue route search using provided labels*/
    void sendContinueRoute(
        int toCluster, int vehID, std::vector<int> clusterSequence, int goalEdge,
        const std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
            &openLabels);

    /** @brief Receive message to continue route search using provided labels*/
    std::tuple<int, std::vector<int>, int,
               std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
    receiveContinueRoute(int fromCluster, const MPI_Status &status);

    // Finished route
    // --------------------------------------------------------------------------------------------------

    /** @brief Send message with completed route*/
    void sendFinishedRoute(int toCluster, int vehID, std::vector<int> clusterSequence,
                           const std::vector<int> &buildPath, const std::vector<double> &priorities,
                           const std::shared_ptr<hierLabel> &rootLabel) const;

    /** @brief Receive message with completed route*/
    std::tuple<int, std::vector<int>, std::vector<int>, std::vector<double>, std::shared_ptr<hierLabel>>
    receiveFinishedRoute(int fromCluster, const MPI_Status &status) const;

    // Route not found
    // -------------------------------------------------------------------------------------------------

    /** @brief Send message that requested route could not be found */
    void sendRouteNotFound(int toCluster, int vehID, std::vector<int> clusterSequence) const;

    /** @brief Receive message that requested route could not be found */
    std::tuple<int, std::vector<int>> receiveRouteNotFound(int fromCluster, const MPI_Status &status) const;

    // Accepted route
    // -------------------------------------------------------------------------------------------------

    /** @brief Send message that requested route could not be found */
    void sendAcceptedRoute(int toCluster, int vehID, const std::vector<double> &priorities,
                           const std::vector<double> &route) const;

    /** @brief Receive message that requested route could not be found */
    std::tuple<int, std::vector<double>, std::vector<double>> receiveAcceptedRoute(int fromCluster,
                                                                                   const MPI_Status &status) const;

    // Found route
    // -------------------------------------------------------------------------------------------------

    /** @brief Send message with found routes */
    void sendFoundRoutes(int toCluster, int vehID, std::vector<int> clusterSequence,
                         const std::vector<std::vector<double>> &routes);

    /** @brief Receive message with found routes */
    std::tuple<int, std::vector<int>, std::vector<std::vector<double>>> receiveFoundRoutes(int fromCluster,
                                                                                           const MPI_Status &status);

    // Activity status for master process
    // ------------------------------------------------------------------------------

    /** @brief Send busy/free status to the master process for synchronization */
    void sendActivityStatus(bool isActive);

    // Labels
    // ----------------------------------------------------------------------------------------------------------

    /** @brief Send labels at the border of the clusters to the neighbor cluster
     * agent */
    void sendLabels(
        const std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
            &labels,
        int destinationCluster);

    /** @brief Receive labels from the neighbor cluster agent for further planning
     */
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
    receiveLabels(int sourceCluster);

    // =================================================================================================================

   public:
    // Initialization
    // ====================================================================================================

    /** @brief Constructor for the Router class */
    Router(int cluster, int numAgents, int intervalLength, Graph &originalGraph, int K,
           TrafficEstimator &trafficEstimator, SpeedEstimator &speedEstimator, FuelEstimator &fuelEstimator);

    /** @brief Update cluster ID and prune excess edges from the cover graphs */
    void updateClusterID(int cluster);

    // Main operations
    // =================================================================================================

    /** @brief Main function for proactive routing */
    std::vector<routeData> proactivelyRoute(
        const std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> &demand, int currentTime);

    /** @brief Synchronize vehicle positions and densities with the master process
     */
    void synchronizePositions(const std::vector<positionData> &currentPositions,
                              const std::unordered_map<int, double> &currentDensities, int currentTime);

    /** @brief Main function for reactive rerouting */
    std::vector<routeData> reactivelyReroute(int currentTime);

    // Getters/setters
    // =================================================================================================

    /** @brief Set the top level limit for the Hierarchical MLS search */
    void setLevelLimit(int levelLimit);

    /** @brief Get the top level limit for the Hierarchical MLS search */
    int getLevelLimit() const;

    /** @brief Get the cover graphs */
    std::vector<std::shared_ptr<CoverGraph>> getCoverGraphs() const;

    /** @brief Get the original graph */
    Graph &getOriginalGraph() const;

    /** @brief Enable or disable logging */
    void setLogging(bool enable) {
        mEnableLogging = enable;
    }

    /** @brief Check if logging is enabled */
    bool isLoggingEnabled() const {
        return mEnableLogging;
    }

    /** @brief Set the epsilon parameter for the MLS search */
    void setEpsilon(double eps) {
        epsilon = eps;
    }

    /** @brief Set the reroute portion */
    void setReroutePortion(double reroutePortion) {
        mReroutePortion = reroutePortion;
    }
};

#endif  // HIERARCHICALSYSTEM_ROUTER_H

