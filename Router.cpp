/**
 * @file Router.cpp
 * @brief Implementation of distributed dynamic routing with hierarchical MLS and MPI.
 */
#include "Router.h"

#include <mpi.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <queue>
#include <random>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

#include "FuelEstimator.h"

namespace std {
template <>
struct hash<std::vector<double>> {
    size_t operator()(const std::vector<double> &v) const {
        size_t seed = 0;
        for (const auto &elem : v) {
            seed ^= std::hash<double>{}(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

template <>
struct hash<std::vector<int>> {
    size_t operator()(const std::vector<int> &v) const {
        std::size_t seed = v.size();
        for (int i : v) {
            seed ^= std::hash<int>{}(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
}  // namespace std

/**
 *
 * @param cluster Cluster/agent ID for message communication
 * @param numAgents Total number of agents in the system
 * @param intervalLength Length of the synchronization interval
 * @param originalGraph Original graph of the road network
 * @param K Number of levels in the hierarchical cover graph
 * @param trafficEstimator Traffic density estimator
 * @param speedEstimator Traffic speed estimator
 * @param fuelEstimator Fuel consumption estimator
 *
 * Main constructor for the Router class
 */
Router::Router(int cluster, int numAgents, int intervalLength, Graph &originalGraph, int K,
               TrafficEstimator &trafficEstimator, SpeedEstimator &speedEstimator, FuelEstimator &fuelEstimator)
    : mOriginalGraph(originalGraph),
      mIntervalLength(intervalLength),
      mTrafficEstimator(trafficEstimator),
      mSpeedEstimator(speedEstimator),
      mFuelEstimator(fuelEstimator) {
    NUM_AGENTS = numAgents;
    mTopLevel = K - 1;
    mLevelLimit = mTopLevel;

    for (int i = 0; i < NUM_AGENTS; ++i) {
        mBitmaskToCluster[1 << i] = i;
    }

    mCoverGraphs = buildInitialCovers(K);
    // Record top levels of nodes
    mNodeLevels.assign(mOriginalGraph.getNumNodes(), -1);
    for (int k = mTopLevel; k >= 0; --k) {
        for (int n : mCoverGraphs[k]->getCoverNodes()) {
            if (mNodeLevels[n] == -1) {
                mNodeLevels[n] = k;
            }
        }
    }

    updateClusterID(cluster);

    std::vector<const Edge *> edgesToPredict;
    std::vector<double> densitiesToPredict;
    std::vector<double> speedsToPredict;
    std::unordered_map<int, int> mapIDs;

    for (auto &e : mCoverGraphs[0]->getCoverEdges()) {
        int innerEdge = e.getInsideEdges()[0];
        mapIDs[originalGraph.getEdge(innerEdge).getID()] = innerEdge;
        mEdgeFlows[innerEdge].assign(60, std::make_pair(0, 0));
        mEdgeDensities[innerEdge].assign(60, 0);
        mEdgeSpeeds[innerEdge].assign(60, originalGraph.getEdge(innerEdge).getSpeedLimit());
        edgesToPredict.push_back(&originalGraph.getEdge(innerEdge));
        densitiesToPredict.push_back(0);
        speedsToPredict.push_back(originalGraph.getEdge(innerEdge).getSpeedLimit());
    }

    std::vector<double> fuels = mFuelEstimator.predictFuelBatch(edgesToPredict, densitiesToPredict, speedsToPredict);
    for (int i = 0; i < edgesToPredict.size(); ++i) {
        mEdgeFuels[mapIDs[edgesToPredict[i]->getID()]].assign(60, fuels[i]);
    }

    // mEdgeFuels[innerEdge].assign(
    // 20, mFuelEstimator.predictFuel(originalGraph.getEdge(innerEdge), 0, mEdgeSpeeds[innerEdge][0]));
}

/**
 *
 * @param time Time in seconds
 * @return Interval number
 *
 * Computes which interval the given time belongs to based on mIntervalLength
 */
int Router::timeToInterval(double time) {

    return ((time - 1) / mIntervalLength) - mPassedIntervals;
    // // Round to closest interval
    // return div(static_cast<int>(time), static_cast<int>(mIntervalLength)).quot - mPassedIntervals;
}

/**
 *
 * @param time Time in seconds
 * @return Interval number
 *
 * Computes which interval the given time belongs to based on mIntervalLength
 */
int Router::timeToInterval(int time) {

    return ((time - 1) / mIntervalLength) - mPassedIntervals;
    // Round to closest interval
    // return div(time, static_cast<int>(mIntervalLength)).quot - mPassedIntervals;
}

/**
 *
 * @param predictedDensities Map of predicted densities for the edges
 * @return Set of congested edges
 *
 * Detects congested edges based on the predicted densities and inner congestion threshold mCongestionThreshold
 */
std::unordered_set<int> Router::predictCongestion(const std::unordered_map<int, double> &predictedDensities) {
    // Use current densities (argument), predicted inflows/outflows (inner records), and edge parameters
    std::unordered_set<int> congestedEdges;

    for (auto &[e, d] : predictedDensities) {
        if (d >= mCongestionThreshold) {
            congestedEdges.insert(e);
        }
    }

    return congestedEdges;
}

/**
 *
 * @param congestedEdges Set of congested edges
 * @return Set of vehicles to be rerouted
 *
 * For every congested edge, performs backward BFS search limited by interval length and randomly selects vehicles
 * to be rerouted based on given rerouting probability
 */
std::unordered_set<int> Router::selectForRerouting(const std::unordered_set<int> &congestedEdges) {
    std::unordered_set<int> vehiclesToReroute;
    // Select vehicles at random for now
    double reroutePortion = mReroutePortion;
    // Iterate over all congested edges
    for (int e : congestedEdges) {
        // Prepare for random selection
        std::random_device rd;
        auto time_seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::seed_seq seed{rd(), static_cast<unsigned int>(time_seed)};
        std::mt19937 gen(seed);
        std::uniform_real_distribution<> dist(0.0, 1.0);

        // Iterate backwards from the congested edge until interval reached
        std::queue<std::pair<int, double>> queue;
        queue.emplace(e, 0);

        while (!queue.empty()) {
            auto [currentEdge, currentTime] = queue.front();
            queue.pop();

            for (int nextEdge : mOriginalGraph.getEdge(currentEdge).getIncomingEdges()) {
                if (congestedEdges.count(e) > 0) {
                    continue;
                }

                double newTime = currentTime + mOriginalGraph.getEdge(nextEdge).getLength() / mEdgeSpeeds[nextEdge][0];
                if (newTime < mIntervalLength) {
                    for (int v : mCurrentEdgePopulation[nextEdge]) {
                        // TODO: think of better selection strategy
                        // Check if the vehicle is expected to go through the congested edge in the next interval
                        auto &fullRoute = mVehicleRoutes[v];
                        int expectedPosition = mVehiclePositions[v];
                        int startTime = mVehicleTimes[v][expectedPosition];

                        int expectedTime = startTime;
                        bool traversesCongestedEdge = false;
                        while (expectedTime < startTime + mIntervalLength) {
                            if (fullRoute[expectedPosition] == e) {
                                traversesCongestedEdge = true;
                                break;
                            }
                            expectedPosition++;
                            if (expectedPosition >= fullRoute.size()) {
                                break;
                            }
                            expectedTime = mVehicleTimes[v][expectedPosition];
                        }
                        if (!traversesCongestedEdge) {
                            continue;
                        }
                        auto reroutingChoice = static_cast<double>(dist(gen));
                        if (reroutingChoice <= reroutePortion) {
                            vehiclesToReroute.insert(v);
                        }
                    }
                    queue.emplace(nextEdge, newTime);
                }
            }
        }
    }

    return vehiclesToReroute;
}

/**
 *
 * @param candidates Set of candidate paths as labels
 * @param priorities Vector of priorities for the planning criteria
 * @return Best route as a label
 *
 * Selects the best route from the candidates based on the given priorities
 * Uses R-method to balance conformity to the planning criteria and general solution quality
 */
std::vector<double> Router::selectBestRoute(std::vector<std::vector<double>> &candidates,
                                            const std::vector<double> &priorities) {
    // Compute weights objective ranks
    size_t numParameters = NUM_PARAMETERS;
    std::vector<double> objectiveWeights(numParameters, 0);

    for (int w = 0; w < numParameters; ++w) {
        objectiveWeights[w] = static_cast<double>(1) / static_cast<double>(w + 1);
    }

    double totalSum = 0;
    std::vector<double> temps(numParameters, 0);
    for (int w = 0; w < numParameters; ++w) {
        double s = 0;
        for (int i = 0; i <= w; ++i) {
            s += objectiveWeights[i];
        }
        temps[w] = static_cast<double>(1) / s;
        totalSum += temps[w];
    }
    objectiveWeights.assign(temps.begin(), temps.end());
    temps.clear();
    for (double &weight : objectiveWeights) {
        weight /= totalSum;
    }

    // Compute weights of solution ranks
    int numSolutionWeights = static_cast<int>(candidates.size());
    std::vector<double> solutionWeights(numSolutionWeights, 0);

    for (int w = 0; w < numSolutionWeights; ++w) {
        solutionWeights[w] = static_cast<double>(1) / static_cast<double>(w + 1);
    }

    totalSum = 0;
    temps.assign(numSolutionWeights, 0);
    for (int w = 0; w < numSolutionWeights; ++w) {
        double s = 0;
        for (int i = 0; i <= w; ++i) {
            s += solutionWeights[i];
        }
        temps[w] = static_cast<double>(1) / s;
        totalSum += temps[w];
    }

    solutionWeights.assign(temps.begin(), temps.end());
    temps.clear();
    for (double &weight : solutionWeights) {
        weight /= totalSum;
    }

    // Create rankings of solutions by parameters
    std::vector<std::set<std::pair<int, std::vector<double>>>> sortingSets(numParameters);
    for (int p = 0; p < numParameters; ++p) {
        for (auto &l : candidates) {
            sortingSets[p].insert(std::make_pair(l[p], l));
        }
    }

    std::unordered_map<std::vector<double>, std::vector<double>> compositeWeights;
    for (auto &l : candidates) {
        compositeWeights[l].assign(numParameters, 0);
    }
    // Compute composite weights for paths

    if (priorities.size() != numParameters || objectiveWeights.size() != numParameters) {
        throw std::invalid_argument("Number of priorities must match the number of parameters");
    }

    for (int p = 0; p < numParameters; ++p) {
        int rank = 0;
        double objWeight = objectiveWeights[static_cast<int>(priorities[p]) - 1];
        for (auto &[value, label] : sortingSets[p]) {
            compositeWeights[label][p] = solutionWeights[rank] * objWeight;
            rank += 1;
        }
    }
    // Select the path with the highest sum of composite weights
    std::vector<double> chosenPath;
    double maxWeight = 0;
    for (auto &c : candidates) {
        double currentWeight = 0;
        for (int p = 0; p < numParameters; ++p) {
            currentWeight += compositeWeights[c][p];
        }
        if (currentWeight > maxWeight) {
            maxWeight = currentWeight;
            chosenPath = c;
        }
    }

    // Select the path with the highest rank of time
    // std::vector<double> chosenPath;
    // auto sol = sortingSets[1].begin();
    // chosenPath = sol->second;
    return chosenPath;
}

std::shared_ptr<hierLabel> Router::selectFastestRoute(
    const std::set<std::shared_ptr<hierLabel>, hierLexCompare> &candidates, const std::vector<double> &priorities) {
    // Find the label with the minimum time
    std::shared_ptr<hierLabel> fastestLabel = nullptr;
    double minTime = std::numeric_limits<double>::max();
    for (const auto &label : candidates) {
        double time = label->parameters[1];
        if (time < minTime) {
            minTime = time;
            fastestLabel = label;
        }
    }
    return fastestLabel;
}

/**
 *
 * @param levelLimit Maximum level to be used for Hierarchical MLS
 *
 * Sets level limit for Hierarchical MLS to plan on
 */
void Router::setLevelLimit(int levelLimit) {
    if (levelLimit > mTopLevel)
        throw std::invalid_argument("Level limit cannot be higher than the top level");
    mLevelLimit = levelLimit;
    logMessage("CONFIG|levelLimit=" + std::to_string(levelLimit));
}

/**
 *
 * @return Level limit for Hierarchical MLS
 */
int Router::getLevelLimit() const {
    return mLevelLimit;
}

/**
 *
 * @return Cover graphs
 */
std::vector<std::shared_ptr<CoverGraph>> Router::getCoverGraphs() const {
    return mCoverGraphs;
}

/**
 *
 * @param goalEdge Goal edge for the search
 * @return All paths from top-level nodes to the goal edge
 *
 * Uses backward "BFS" search to find all paths from top-level nodes to the goal edge
 * The returned labels have arbitrary parameter vectors and should not be used for parameter computation
 */
std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
Router::backwardHierarchicalSearch(int goalEdge) {
    // Run backward search, do not expand from nodes on the upper level
    // Identify the highest level where the goal node is present

    /// The labels do not contain real parameters since those are dynamic and only can be computed forward
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> closedLabels;
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> openLabels;
    std::unordered_map<int, std::unordered_map<int, std::set<std::vector<double>>>> tSets;

    int goalNode = mOriginalGraph.getEdge(goalEdge).getStartNode();
    const CoverEdge &goalCoverEdge = mCoverGraphs[0]->originalToCoverEdge(goalEdge);

    std::set<std::shared_ptr<hierLabel>, hierLexCompare> toExpand;

    std::vector<double> initialParameters(NUM_PARAMETERS, 1);

    auto pGoalLabel = std::make_shared<hierLabel>(goalNode, initialParameters, nullptr,
                                                  std::make_shared<CoverEdge>(goalCoverEdge), 1);
    toExpand.insert(pGoalLabel);
    openLabels[goalNode][goalEdge].insert(pGoalLabel);

    // TODO: replace priority queue to conform to BFS or DFS and be more efficient

    while (!toExpand.empty()) {
        auto pCurrentLabel = *toExpand.begin();
        toExpand.erase(toExpand.begin());
        int currentNode = pCurrentLabel->node;
        int currentLevel = std::min(mNodeLevels[currentNode], mLevelLimit);
        std::vector<double> currentParameters = pCurrentLabel->parameters;
        int outgoingEdge = pCurrentLabel->usedEdge->getStartEdge();

        openLabels[currentNode][outgoingEdge].erase(pCurrentLabel);
        closedLabels[currentNode][outgoingEdge].insert(pCurrentLabel);
        updateTSet(tSets[currentNode][outgoingEdge], pCurrentLabel);

        if (currentLevel == mLevelLimit)
            continue;

        for (int edge : mCoverGraphs[currentLevel]->getConnectedIncomingEdges(outgoingEdge)) {
            const CoverEdge &rPreviousEdge = mCoverGraphs[currentLevel]->getCoverEdge(edge);

            int previousNode = rPreviousEdge.getStartNode();
            int nextOutgoingEdge = rPreviousEdge.getStartEdge();

            std::vector<double> prefixParameters(NUM_PARAMETERS, currentParameters[0] + 1);

            std::vector<double> newParameters =
                linkPaths(prefixParameters, rPreviousEdge.getNumHops(), currentParameters, pCurrentLabel->numHops);

            // Do not perform domination check, since the parameters are not real

            // if (tDiscard(tSets[previousNode][nextOutgoingEdge], newParameters))
            //     continue;
            //
            // bool isDominated = false;
            // std::stack<std::shared_ptr<hierLabel>> toRemove;
            // for (auto &rLab: openLabels[previousNode][nextOutgoingEdge]) {
            //     if (checkDomination(rLab->parameters, newParameters)) {
            //         isDominated = true;
            //         break;
            //     } else if (checkDomination(newParameters, rLab->parameters)) {
            //         toRemove.push(rLab);
            //     }
            // }
            // if (isDominated)
            //     continue;
            // while (!toRemove.empty()) {
            //     auto pLab = toRemove.top();
            //     toRemove.pop();
            //     openLabels[previousNode][nextOutgoingEdge].erase(pLab);
            //     toExpand.erase(pLab);
            // }

            auto pNewLabel = std::make_shared<hierLabel>(previousNode, newParameters, pCurrentLabel,
                                                         std::make_shared<CoverEdge>(rPreviousEdge),
                                                         pCurrentLabel->numHops + rPreviousEdge.getNumHops());
            openLabels[previousNode][nextOutgoingEdge].insert(pNewLabel);
            toExpand.insert(pNewLabel);
        }
    }

    return closedLabels;
}

/**
 *
 * @param rTSet current tSet of truncated vectors
 * @param rNewVec full candidate vector to be checked for domination against tSet
 * @return True if the new vector is dominated by any vector in tSet, false otherwise
 *
 * t-discarding domination check with epsilon-pruning enabled
 */
bool Router::tDiscard(std::set<std::vector<double>> &rTSet, std::vector<double> &rNewVec) const {
    if (rTSet.empty()) {
        return false;
    }
    for (const std::vector<double> &t : rTSet) {
        bool dominating = true;
        for (int i = 0; i < t.size(); ++i) {
            if (epsilon * t[i] > rNewVec[i + 1]) {
                dominating = false;
                break;
            }
        }
        if (dominating) {
            return true;
        }
    }

    return false;
}

/**
 *
 * @param rTSet current tSet of truncated vectors
 * @param pLab new label to be potentially added to tSet
 *
 * Update current tSet.
 * If the truncated vector of the new label is not dominated by any vector in tSet, add it to tSet and remove all
 * dominated vectors
 */
void Router::updateTSet(std::set<std::vector<double>> &rTSet, const std::shared_ptr<hierLabel> &pLab) {
    const std::vector<double> new_candidate(pLab->parameters.begin() + 1, pLab->parameters.end());
    std::stack<std::vector<double>> toRemove;
    for (const std::vector<double> &t : rTSet) {
        if (checkDomination(t, new_candidate)) {
            return;
        } else if (checkDomination(new_candidate, t)) {
            toRemove.push(t);
        }
    }

    while (!toRemove.empty()) {
        const std::vector<double> t = toRemove.top();
        toRemove.pop();
        rTSet.erase(t);
    }
    rTSet.insert(new_candidate);
}

/**
 *
 * @param rVector1 vector to dominated
 * @param rVector2 vector to be dominated
 * @return True if rVector1 dominates rVector2, false otherwise
 *
 * Standard domination check with epsilon pruning enabled
 */
bool Router::checkDomination(const std::vector<double> &rVector1, const std::vector<double> &rVector2) const {
    for (int i = 0; i < rVector1.size(); ++i) {
        if (epsilon * rVector1[i] > rVector2[i])
            return false;
    }
    return true;
}

/**
 *
 * @param rCurrentCriteria criteria vector of the current path
 * @param numOriginalEdges1 number of original edges ("hops") in the current path
 * @param rSuffixCriteria criteria vector the path is extended by
 * @param numOriginalEdges2 number of original edges ("hops") in the path extension
 * @return criteria vector of the extended path
 *
 * Computes criteria vector of the extended path.
 * Considered criteria are (distance, traversal time, fuel consumption, average density)
 * The first three criteria are summed, density is weighted-averaged
 */
std::vector<double> Router::linkPaths(const std::vector<double> &rCurrentCriteria, int numOriginalEdges1,
                                      const std::vector<double> &rSuffixCriteria, int numOriginalEdges2) {
    std::vector<double> fullCriteria(rCurrentCriteria.size(), 0);
    for (int i = 0; i < fullCriteria.size(); ++i) {
        fullCriteria[i] = rCurrentCriteria[i] + rSuffixCriteria[i];
    }

    // fullCriteria.back() = (numOriginalEdges1 * rCurrentCriteria.back() + numOriginalEdges2 * rSuffixCriteria.back())
    // /
    //                       (numOriginalEdges1 + numOriginalEdges2);

    return fullCriteria;
}

/**
 *
 * @return Original graph
 */
Graph &Router::getOriginalGraph() const {
    return mOriginalGraph;
}

/**
 *
 * @param newCluster New cluster ID
 *
 * Function that is called during Router initialization.
 * Prunes cover edges that do not belong to the current cluster.
 * This is done to ensure that each Router agent has info on all cover nodes in the scenario, but only own cover edges
 */
void Router::updateClusterID(int newCluster) {
    mClusterID = newCluster;
    logMessage("CONFIG|clusterID=" + std::to_string(newCluster));
    // Process ground cover independently, then others in a loop
    mCoverGraphs[0]->updateClusterID(newCluster);
    for (size_t t = 1; t < mCoverGraphs.size(); ++t) {
        mCoverGraphs[t]->setClusterID(newCluster);
        mCoverGraphs[t]->buildCoverEdges();
    }
}

/**
 *
 * @param k Number of cover graph levels to be built
 * @return Cover graphs
 *
 * Hierarchical cover graph construction function. Builds full cover graphs that will be pruned cluster-wise later.
 * Called at Router initialization
 * Number of levels is determined by the parameter k and the minimum ratio of vertices left in the cover
 */
std::vector<std::shared_ptr<CoverGraph>> Router::buildInitialCovers(int k) {
    std::vector<std::shared_ptr<CoverGraph>> coverGraphs;
    coverGraphs.reserve(k);
    coverGraphs.push_back(std::make_shared<CoverGraph>(mOriginalGraph));
    int initialNumVertices = mOriginalGraph.getNumNodes();
    for (int i = 1; i < k; ++i) {
        //        std::cout << "Cover " << i << std::endl;
        coverGraphs.push_back(std::make_shared<CoverGraph>(coverGraphs[i - 1]));
        double remainingVertexPercentage =
            (double) coverGraphs[i]->getNumNodes() / static_cast<double>(initialNumVertices);
        if (remainingVertexPercentage < 0.01) {
            break;
        }
    }

    return coverGraphs;
}

template <typename T>
std::string vectorToString(const std::vector<T> &vec, const std::string &delimiter = ", ") {
    std::ostringstream oss;

    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << delimiter;
        }
    }
    return oss.str();
}

// Overload for a vector of chars to produce a string directly
std::string vectorToString(const std::vector<char> &vec, const std::string &delimiter = "") {
    return std::string(vec.begin(), vec.end());
}

std::string labelToString(std::shared_ptr<hierLabel> &label) {
    std::ostringstream oss;
    oss << label->node;
    oss << "; ";
    oss << vectorToString(label->parameters);
    oss << "; ";
    oss << label->usedEdge->getId();
    return oss.str();
}

/**
 *
 * @param demand Demand data of newly connected vehicles (id, current route, priorities)
 * @param currentTime Current time in the simulation
 * @return Assigned routes for the vehicles
 *
 * Main function for proactive routing.
 * Receives only demand of vehicles that start/are in its own cluster.
 * Computes routes for own vehicles. If they end in different clusters, sends them further to the best cluster
 * Afterward, operates in a loop:
 * 1. Process all pending requests
 * 2. Receive incoming messages if there are any
 * 3. Check if global termination condition is met
 *
 * Best cluster to send request to is determined by the number of open nodes in the query
 * If the current best cluster finds a route, accept it and send back
 * If the current best cluster does not find a route, send the request to the next best cluster
 * Imitates tree-like search with backtracking among clusters
 */
std::vector<routeData> Router::proactivelyRoute(
    const std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> &demand, int currentTime) {
    /*
     * Receive demand data of newly connected vehicles
     * Plan routes for them and take note of the assigned routes
     */
    mCurrentTime = currentTime;
    currentTime -= simulationStart;
    std::vector<routeData> assignedRoutes;
    std::unordered_map<
        int,
        std::unordered_map<
            std::vector<int>,
            std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>>
        storedLabels;
    std::unordered_map<int, std::vector<double>> storedPriorities;  // Priorities of the current solution
    std::unordered_set<int> expectedReturns;  // Vehicles that are expected to return to the current cluster
    std::unordered_map<int, int> goalEdges;   // Goal edge of the current solution
    std::unordered_map<int, std::vector<int>>
        defaultRoutes;  // Default routes for vehicles that end in different clusters, used in case of no solution found
    std::unordered_map<int, std::unordered_map<std::vector<int>, bool>> exploredSequences;

    bool isActive = true;

    // For every vehicle, plan a route
    for (const auto &[vehID, route, priorities] : demand) {
        int startEdge = route[0];
        int goalEdge = route[route.size() - 1];

        logMessage("ROUTE|demand|veh=" + std::to_string(vehID));

        mCurrentEdgePopulation[startEdge].insert(vehID);
        mVehiclePositions[vehID] = 0;

        // Check if the goal edge is in the current cluster
        if (mOriginalGraph.getEdge(goalEdge).inCluster(mClusterID)) {
            // If it is, plan with finishing connection to the goal vertex

            std::set<std::shared_ptr<hierLabel>, hierLexCompare> finishedPaths =
                completeHierarchicalSearch(startEdge, goalEdge);
            if (finishedPaths.empty()) {
                recordRoute(vehID, route, std::vector<double>(), priorities, currentTime);
                std::vector<int> buildPath;
                assignedRoutes.emplace_back(vehID, buildPath);
                logMessage("ROUTE|abandon|veh=" + std::to_string(vehID));
                continue;
            }
            // Choose the path according to the priorities
            std::vector<std::vector<double>> finishedPathsVec;
            for (auto &path : finishedPaths) {
                finishedPathsVec.push_back(path->parameters);
            }
            std::vector<double> chosenPath = selectBestRoute(finishedPathsVec, priorities);
            std::shared_ptr<hierLabel> chosenPathLabel;
            for (auto &path : finishedPaths) {
                if (path->parameters == chosenPath) {
                    chosenPathLabel = path;
                    break;
                }
            }

            std::vector<int> buildPath;
            std::shared_ptr<hierLabel> rootLabel = buildForwardPath(chosenPathLabel, buildPath);
            int startTime = rootLabel->parameters[1];
            // Store the path as completed
            buildPath.insert(buildPath.begin(), rootLabel->usedEdge->getInsideEdges().begin(),
                             rootLabel->usedEdge->getInsideEdges().end());
            assignedRoutes.emplace_back(vehID, buildPath);
            // Record the route to inner expectations
            recordRoute(vehID, buildPath, std::vector<double>(), priorities, currentTime);
        } else {
            // The start and the goal are in different areas, use distributed planning
            std::vector<int> clusterSequence({mClusterID});
            std::unordered_set<int> extendableClusters = findExtendableClusters(startEdge, goalEdge, clusterSequence);
            auto openLabels = distributedHierarchicalSearch(startEdge, goalEdge, extendableClusters);
            if (openLabels.empty()) {
                recordRoute(vehID, route, std::vector<double>(), priorities, currentTime);
                std::vector<int> buildPath;
                assignedRoutes.emplace_back(vehID, buildPath);
                logMessage("ROUTE|abandon|veh=" + std::to_string(vehID));
                continue;
            }
            // Send created labels further to neighbor ACUs
            // Sort existing labels by their respective clusters

            std::unordered_map<
                int,
                std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
                splitByClusters;
            for (auto &[node, p] : openLabels) {
                for (auto &[edge, labels] : p) {
                    // Check which other cluster the node belongs to
                    for (int i = 0; i < NUM_AGENTS; ++i) {
                        if (i != mClusterID && mOriginalGraph.getEdge(edge).inCluster(i)) {
                            splitByClusters[i][node][edge] = labels;
                            std::vector<int> seq({mClusterID, i});
                            exploredSequences[vehID][seq] = false;
                        }
                    }
                }
            }

            // Select an unexplored cluster to forward the request to
            std::vector<int> nextSequence;
            for (auto &[sequence, explored] : exploredSequences[vehID]) {
                if (!explored) {
                    nextSequence = sequence;
                    break;
                }
            }

            if (!nextSequence.empty()) {
                // Send request info
                sendContinueRoute(nextSequence.back(), vehID, nextSequence, goalEdge,
                                  splitByClusters[nextSequence.back()]);
                expectedReturns.insert(vehID);
                goalEdges[vehID] = goalEdge;
                storedPriorities[vehID] = priorities;
                defaultRoutes[vehID] = route;
                for (auto &[cluster, p] : splitByClusters) {
                    std::vector<int> seq({mClusterID, cluster});
                    storedLabels[vehID][seq] = splitByClusters[cluster];
                }
            } else {
                recordRoute(vehID, route, std::vector<double>(), priorities, currentTime);
                std::vector<int> buildPath;
                assignedRoutes.emplace_back(vehID, buildPath);
                logMessage("ROUTE|abandon|veh=" + std::to_string(vehID));
            }
        }
    }

    std::unordered_map<int, std::vector<std::vector<double>>>
        aggregatedSolutions;  // Aggregated solutions for vehicles that end in different clusters

    std::queue<std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
        extensionQueue;
    std::queue<std::tuple<int, int, std::vector<int>, int>> extensionSource;
    std::unordered_map<int, std::unordered_map<std::vector<int>, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
        completedLabels;

    while (true) {
        // 1. Check if there are tasks in the local queue
        if (!extensionQueue.empty()) {
            auto openLabels = extensionQueue.front();
            auto [vehID, sourceCluster, clusterSequence, goalEdge] = extensionSource.front();
            extensionQueue.pop();
            extensionSource.pop();

            // Check if the query is completable (the goal edge is in the current cluster)
            if (mOriginalGraph.getEdge(goalEdge).inCluster(mClusterID)) {
                // Complete the query
                std::set<std::shared_ptr<hierLabel>, hierLexCompare> finishedPaths =
                    completeHierarchicalSearch(-1, goalEdge, openLabels);
                // receivedQueryFrom.erase(vehID);
                if (!finishedPaths.empty()) {
                    // Pack paths into a vector of vectors of parameters
                    std::vector<std::vector<double>> paths;
                    for (auto &path : finishedPaths) {
                        paths.push_back(path->parameters);
                    }
                    std::vector<int> seq(clusterSequence);
                    seq.pop_back();
                    // Send the paths back to the previous cluster
                    sendFoundRoutes(sourceCluster, vehID, seq, paths);
                    // Store the found labels for later processing
                    completedLabels[vehID][clusterSequence] = finishedPaths;
                    // Clear the open labels
                    openLabels.clear();

                } else {
                    // No viable root was found, return back failure message to the cluster
                    clusterSequence.pop_back();
                    sendRouteNotFound(sourceCluster, vehID, clusterSequence);
                }
            } else {
                // The start and the goal are in different areas, use distributed planning
                std::unordered_set<int> extendableClusters =
                    findExtendableClusters(-1, goalEdge, clusterSequence, openLabels);
                auto extendedLabels = distributedHierarchicalSearch(-1, goalEdge, extendableClusters, openLabels);
                // If no continuation was found, return back failure message to the cluster
                if (extendedLabels.empty()) {
                    clusterSequence.pop_back();
                    sendRouteNotFound(sourceCluster, vehID, clusterSequence);
                    continue;
                }

                // Send created labels further to neighbor ACUs
                // Sort existing labels by their respective clusters

                std::unordered_map<
                    int, std::unordered_map<
                             int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
                    splitByClusters;

                for (auto &[node, p] : extendedLabels) {
                    for (auto &[edge, labels] : p) {
                        // Check which other cluster the node belongs to
                        for (int i = 0; i < NUM_AGENTS; ++i) {
                            if (mOriginalGraph.getEdge(edge).inCluster(i)) {
                                if (std::find(clusterSequence.begin(), clusterSequence.end(), i) !=
                                    clusterSequence.end()) {
                                    continue;
                                }
                                splitByClusters[i][node][edge] = labels;
                                std::vector<int> seq(clusterSequence);
                                seq.push_back(i);
                                exploredSequences[vehID][seq] = false;
                            }
                        }
                    }
                }

                // Select the best cluster to forward the request to bu the number of nodes
                std::vector<int> nextSequence;
                for (auto &[cluster, p] : splitByClusters) {
                    std::vector<int> seq(clusterSequence);
                    seq.push_back(cluster);
                    if (!exploredSequences[vehID][seq]) {
                        nextSequence = seq;
                        break;
                    }
                }
                if (!nextSequence.empty()) {
                    // Send request info
                    sendContinueRoute(nextSequence.back(), vehID, nextSequence, goalEdge,
                                      splitByClusters[nextSequence.back()]);
                    expectedReturns.insert(vehID);
                    goalEdges[vehID] = goalEdge;
                    for (auto &[cluster, p] : splitByClusters) {
                        std::vector<int> seq(clusterSequence);
                        seq.push_back(cluster);
                        storedLabels[vehID][seq] = splitByClusters[cluster];
                    }
                } else {
                    // No more clusters to forward the request to, return back failure message to the cluster
                    clusterSequence.pop_back();
                    sendRouteNotFound(sourceCluster, vehID, clusterSequence);
                }
            }
        }

        // 2. Check for incoming tasks or requests (non-blocking)
        for (int n = 0; n < NUM_AGENTS; ++n) {
            if (n == mClusterID) {
                continue;
            }
            MPI_Status status;
            int continueFlag;
            MPI_Iprobe(n + 1, static_cast<int>(MessageTags::continueRoute), MPI_COMM_WORLD, &continueFlag, &status);
            // std::cout << "ACU " << mClusterID << " probed from ACU " << n << " tag continueRoute" << std::endl;
            if (continueFlag) {
                //                std::cout << "Received a message" << std::endl;
                // A task is received, process it
                // Received a forwarded task - add it to the queue for processing
                auto [vehID, clusterSequence, goalEdge, newLabels] = receiveContinueRoute(n, status);
                // Prevent message cycling by discarding already processed routes
                std::vector<int> seq(clusterSequence);
                seq.pop_back();
                if (std::find(seq.begin(), seq.end(), mClusterID) != seq.end()) {
                    sendRouteNotFound(n, vehID, clusterSequence);
                    continue;
                }

                extensionQueue.push(newLabels);
                extensionSource.emplace(vehID, n, clusterSequence, goalEdge);
                // Notify master process about activity
                if (!isActive) {
                    isActive = true;
                    sendActivityStatus(isActive);
                }
            }
            int finishedFlag;
            MPI_Iprobe(n + 1, static_cast<int>(MessageTags::finishedRoute), MPI_COMM_WORLD, &finishedFlag, &status);
            // std::cout << "ACU " << mClusterID << " probed from ACU " << n << " tag finishedRoute" << std::endl;
            if (finishedFlag) {

                // Received a completed route
                // Continue building and send further if not the source cluster
                // Continue building and return if source cluster
                auto [vehID, clusterSequence, buildPath, priors, receivedLabel] = receiveFinishedRoute(n, status);

                // Find the corresponding label among stored
                std::shared_ptr<hierLabel> correspondingLabel;
                expectedReturns.erase(vehID);

                std::vector<int> seq(clusterSequence);
                seq.push_back(n);

                for (auto &[edge, labels] : storedLabels[vehID][seq][receivedLabel->node]) {
                    auto it = std::find_if(labels.begin(), labels.end(),
                                           [&rl = receivedLabel](const std::shared_ptr<hierLabel> &l) -> bool {
                                               return rl->parameters == l->parameters;
                                           });
                    if (it != labels.end()) {
                        correspondingLabel = *it;
                        break;
                    }
                }

                // Extend the path until the borders of own cluster or query end
                std::shared_ptr<hierLabel> rootLabel = buildForwardPath(correspondingLabel, buildPath);
                int startTime = static_cast<int>(rootLabel->parameters[1]);

                storedLabels.erase(vehID);
                goalEdges.erase(vehID);
                aggregatedSolutions.erase(vehID);
                exploredSequences.erase(vehID);
                storedPriorities.erase(vehID);
                defaultRoutes.erase(vehID);

                if (clusterSequence.size() > 1) {
                    // The route was received from someone else, send it further back
                    // Find the source which sent the request originally
                    // Add the border edge to recording
                    std::vector<int> myPath;
                    myPath.push_back(rootLabel->usedEdge->getInsideEdges()[0]);
                    myPath.insert(myPath.end(), buildPath.begin(), buildPath.end());
                    std::vector<double> myTimes({static_cast<double>(currentTime + startTime)});
                    recordRoute(vehID, myPath, myTimes, priors, currentTime + startTime);

                    // Extract the source cluster from the sequence as the third element from the back
                    int redirectTo = clusterSequence[clusterSequence.size() - 2];
                    clusterSequence.pop_back();
                    // Send it further
                    sendFinishedRoute(redirectTo, vehID, clusterSequence, buildPath, priors, rootLabel);
                } else {
                    // The route is own, add it to the completed routes
                    buildPath.insert(buildPath.begin(), rootLabel->usedEdge->getInsideEdges().begin(),
                                     rootLabel->usedEdge->getInsideEdges().begin() + 1);
                    recordRoute(vehID, buildPath, std::vector<double>(), priors, currentTime);

                    assignedRoutes.emplace_back(vehID, buildPath);
                }
            }
            int notFoundFlag;
            MPI_Iprobe(n + 1, static_cast<int>(MessageTags::routeNotFound), MPI_COMM_WORLD, &notFoundFlag, &status);
            // std::cout << "ACU " << mClusterID << " probed from ACU " << n << " tag routeNotFound" << std::endl;
            if (notFoundFlag) {
                // Received a message that the route was not found
                auto [vehID, clusterSequence] = receiveRouteNotFound(n, status);
                // Find another cluster to send the request to
                std::vector<int> seq(clusterSequence);
                seq.push_back(n);
                exploredSequences[vehID][seq] = true;
                storedLabels[vehID].erase(seq);
                seq.clear();

                // Check if there is another cluster to send the request to
                std::vector<int> nextSequence;
                for (auto &[sequence, explored] : exploredSequences[vehID]) {
                    if (!explored) {
                        nextSequence = sequence;
                        break;
                    }
                }

                if (!nextSequence.empty()) {
                    sendContinueRoute(nextSequence.back(), vehID, nextSequence, goalEdges[vehID],
                                      storedLabels[vehID][nextSequence]);
                } else {
                    // If own request, check for aggregated solutions
                    if (clusterSequence.size() == 1) {
                        if (!aggregatedSolutions[vehID].empty()) {
                            std::vector<double> bestParameters =
                                selectBestRoute(aggregatedSolutions[vehID], storedPriorities[vehID]);
                            // Find the target cluster
                            if (mOriginalGraph.getEdge(goalEdges[vehID]).isBorderEdge()) {
                                // Check if the edge belongs to the own cluster
                                if (mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() & (1 << mClusterID)) {
                                    int targetCluster =
                                        mBitmaskToCluster[mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() &
                                                          ~(1 << mClusterID)];
                                    sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
                                } else {
                                    // Extract both clusters from the edge bitmask
                                    int cluster1 =
                                        __builtin_ctz(mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask());
                                    int cluster2 =
                                        __builtin_ctz(mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() &
                                                      ~(1 << cluster1));
                                    sendAcceptedRoute(cluster1, vehID, storedPriorities[vehID], bestParameters);
                                    sendAcceptedRoute(cluster2, vehID, storedPriorities[vehID], bestParameters);
                                }
                            } else {
                                // The edge is not a border edge, send the route to the target cluster
                                int targetCluster = mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask();
                                targetCluster = mBitmaskToCluster[targetCluster];
                                sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
                            }
                        } else {
                            // No aggregated solutions, abandon the vehicle
                            recordRoute(vehID, defaultRoutes[vehID], std::vector<double>(), storedPriorities[vehID],
                                        currentTime);
                            std::vector<int> buildPath;
                            assignedRoutes.emplace_back(vehID, buildPath);
                            expectedReturns.erase(vehID);
                            logMessage("ABANDON|vehicle=" + std::to_string(vehID));
                        }
                    } else {
                        if (!aggregatedSolutions[vehID].empty()) {
                            std::vector<int> seq(clusterSequence);
                            seq.pop_back();
                            sendFoundRoutes(seq.back(), vehID, seq, aggregatedSolutions[vehID]);
                            aggregatedSolutions.erase(vehID);
                            // Clear the explored clusters
                            exploredSequences.erase(vehID);
                            // Clear the stored labels
                            expectedReturns.erase(vehID);
                            goalEdges.erase(vehID);
                        } else {
                            // No aggregated solutions, send back failure message
                            std::vector<int> seq(clusterSequence);
                            seq.pop_back();
                            sendRouteNotFound(seq.back(), vehID, seq);
                            aggregatedSolutions.erase(vehID);
                            exploredSequences.erase(vehID);
                            // storedLabels.erase(vehID);
                            expectedReturns.erase(vehID);
                            goalEdges.erase(vehID);
                        }
                    }
                }
            }

            int foundRoutesFlag;
            MPI_Iprobe(n + 1, static_cast<int>(MessageTags::foundRoutes), MPI_COMM_WORLD, &foundRoutesFlag, &status);
            if (foundRoutesFlag) {
                // The message is sent by a cluster to a previous one when it finds or completes all routes
                // Received a message that the route was found
                // Process incoming criteria
                auto [vehID, clusterSequence, foundPaths] = receiveFoundRoutes(n, status);
                std::vector<int> seq(clusterSequence);
                seq.push_back(n);
                exploredSequences[vehID][seq] = true;
                // Process the paths
                aggregatedSolutions[vehID].insert(aggregatedSolutions[vehID].end(), foundPaths.begin(),
                                                  foundPaths.end());
                // Look for the next cluster to forward the request to
                std::vector<int> nextSequence;
                for (auto &[sequence, explored] : exploredSequences[vehID]) {
                    if (!explored) {
                        nextSequence = sequence;
                        break;
                    }
                }
                if (!nextSequence.empty()) {
                    sendContinueRoute(nextSequence.back(), vehID, nextSequence, goalEdges[vehID],
                                      storedLabels[vehID][nextSequence]);

                } else {
                    if (clusterSequence.size() > 1) {
                        // Send all aggregated solutions to the previous cluster
                        std::vector<int> seq(clusterSequence);
                        seq.pop_back();
                        sendFoundRoutes(seq.back(), vehID, seq, aggregatedSolutions[vehID]);
                        aggregatedSolutions.erase(vehID);
                        // Clear the explored clusters
                        exploredSequences.erase(vehID);
                        // Clear the stored labels
                        expectedReturns.erase(vehID);
                        goalEdges.erase(vehID);
                    } else {
                        // The routes returned to the starting cluster, select one and send it to the target cluster
                        std::vector<double> bestParameters =
                            selectBestRoute(aggregatedSolutions[vehID], storedPriorities[vehID]);
                        // Find the target cluster
                        if (mOriginalGraph.getEdge(goalEdges[vehID]).isBorderEdge()) {
                            // Check if the edge belongs to the own cluster
                            if (mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() & (1 << mClusterID)) {
                                int targetCluster =
                                    mBitmaskToCluster[mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() &
                                                      ~(1 << mClusterID)];
                                sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
                            } else {
                                // Extract both clusters from the edge bitmask
                                int cluster1 =
                                    __builtin_ctz(mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask());
                                int cluster2 = __builtin_ctz(
                                    mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() & ~(1 << cluster1));

                                int targetCluster = -1;
                                // Check incoming edges for cluster bitmask
                                std::unordered_set<int> incomingEdges =
                                    mOriginalGraph.getEdge(goalEdges[vehID]).getIncomingEdges();
                                for (int edge : incomingEdges) {
                                    if (!mOriginalGraph.getEdge(edge).isBorderEdge()) {
                                        if (mOriginalGraph.getEdge(edge).getClusterBitmask() & (1 << cluster1)) {
                                            targetCluster = cluster1;
                                            break;
                                        } else if (mOriginalGraph.getEdge(edge).getClusterBitmask() & (1 << cluster2)) {
                                            targetCluster = cluster2;
                                            break;
                                        }
                                    }
                                }
                                if (targetCluster != -1) {
                                    sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
                                } else {
                                    sendAcceptedRoute(cluster1, vehID, storedPriorities[vehID], bestParameters);
                                    sendAcceptedRoute(cluster2, vehID, storedPriorities[vehID], bestParameters);
                                }
                            }
                        } else {
                            // The edge is not a border edge, send the route to the target cluster
                            int targetCluster = mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask();
                            targetCluster =
                                mBitmaskToCluster[mOriginalGraph.getEdge(goalEdges[vehID]).getClusterBitmask() &
                                                  ~(1 << mClusterID)];
                            sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
                        }
                    }
                }
            }

            int acceptedRouteFlag;
            MPI_Iprobe(n + 1, static_cast<int>(MessageTags::acceptedRoute), MPI_COMM_WORLD, &acceptedRouteFlag,
                       &status);
            if (acceptedRouteFlag) {
                // The message is sent by the starting cluster to the one which found the routes to be accepted
                auto [vehID, priorities, acceptedRoute] = receiveAcceptedRoute(n, status);
                // Find the corresponding label among stored by comparing the parameters
                std::shared_ptr<hierLabel> correspondingLabel;
                std::vector<int> correspondingSequence;
                for (auto &[sequence, labels] : completedLabels[vehID]) {
                    for (auto &label : labels) {
                        if (label->parameters == acceptedRoute) {
                            correspondingLabel = label;
                            correspondingSequence = sequence;
                            break;
                        }
                    }
                }

                if (correspondingLabel == nullptr) {
                    // Check if the label exists but double imprecision is present
                    for (auto &[sequence, labels] : completedLabels[vehID]) {
                        for (auto &label : labels) {
                            bool found = true;
                            for (int i = 0; i < NUM_PARAMETERS; ++i) {
                                if (fabs(label->parameters[i] - acceptedRoute[i]) > 0.000001) {
                                    found = false;
                                    break;
                                }
                            }
                            if (found) {
                                correspondingLabel = label;
                                correspondingSequence = sequence;
                                break;
                            }
                        }
                    }
                }

                if (correspondingLabel == nullptr) {
                    // Check if the goal edge is a border edge
                    if (goalEdges.count(vehID) > 0) {
                        if (mOriginalGraph.getEdge(goalEdges[vehID]).isBorderEdge()) {
                            // The label is not own, ignore the message
                            continue;
                        }
                    } else {
                        logError("Received accepted route from ACU " + std::to_string(n) +
                                 " but no corresponding label found");
                        continue;
                    }
                }
                // Extend the path until the borders of own cluster or query end
                std::vector<int> buildPath;
                std::shared_ptr<hierLabel> rootLabel = buildForwardPath(correspondingLabel, buildPath);
                // Send the accepted route back to the previous cluster
                // Find the source cluster by the border edge
                std::vector<int> sourceSequence(correspondingSequence);
                sourceSequence.pop_back();
                sendFinishedRoute(sourceSequence.back(), vehID, sourceSequence, buildPath, priorities, rootLabel);
                // Clear the stored labels
                completedLabels.erase(vehID);
                expectedReturns.erase(vehID);

                // std::shared_ptr<hierLabel> chosenPath = selectBestRoute(finishedPaths, priorities);
                // std::vector<int> buildPath;
                // std::shared_ptr<hierLabel> rootLabel = buildForwardPath(chosenPath, buildPath);
                int startTime = static_cast<int>(rootLabel->parameters[1]);
                // Record the backward path of the route
                // Add border edge to recording
                std::vector<int> myPath;
                myPath.push_back(rootLabel->usedEdge->getInsideEdges()[0]);
                myPath.insert(myPath.end(), buildPath.begin(), buildPath.end());
                std::vector<double> myTimes({static_cast<double>(currentTime + startTime)});
                recordRoute(vehID, myPath, myTimes, priorities, currentTime + startTime);
            }
        }

        // 3. Periodic Check for Termination Condition
        if (extensionQueue.empty() && expectedReturns.empty() && isActive) {
            // Mark process as finished and notify master
            isActive = false;
            sendActivityStatus(isActive);
            logMessage("SEND|finished|to=MASTER|tag=processFinished");
        }

        // Check for end signal from the master process
        if (!isActive) {
            // Check for message
            MPI_Status status;
            int flag;
            MPI_Iprobe(0, static_cast<int>(MessageTags::processFinished), MPI_COMM_WORLD, &flag, &status);
            // std::cout << "ACU " << mClusterID << " probed message from MASTER tag processFinished" << std::endl;
            if (flag) {
                int d;
                MPI_Recv(&d, 1, MPI_INT, 0, static_cast<int>(MessageTags::processFinished), MPI_COMM_WORLD,
                         MPI_STATUS_IGNORE);
                logMessage("RECV|finished|from=MASTER|tag=processFinished");
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return assignedRoutes;
}

std::unordered_set<int> Router::findExtendableClusters(
    int startEdge, int goalEdge, std::vector<int> &clusterSequence,
    std::optional<
        std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
        openLabels) const {
    // Use ellipse pruning to limit extendable clusters
    std::unordered_set<int> extendableClusters;
    std::pair<double, double> o;
    if (startEdge != -1) {
        o = mOriginalGraph.getNode(mOriginalGraph.getEdge(startEdge).getStartNode()).getCoordinates();
    } else {
        o = std::make_pair(0, 0);
        for (auto &[node, p] : openLabels.value()) {
            o.first += mOriginalGraph.getNode(node).getCoordinates().first;
            o.second += mOriginalGraph.getNode(node).getCoordinates().second;
        }
        o.first /= openLabels.value().size();
        o.second /= openLabels.value().size();
    }

    auto d = mOriginalGraph.getNode(mOriginalGraph.getEdge(goalEdge).getEndNode()).getCoordinates();

    double coeff = 0.1;
    double od = sqrt(pow(o.first - d.first, 2) + pow(o.second - d.second, 2));
    double lamb = od * coeff;

    for (int n : mCoverGraphs[0]->getCoverNodes()) {
        std::pair<double, double> v = mOriginalGraph.getNode(n).getCoordinates();
        double ov = sqrt(pow(o.first - v.first, 2) + pow(o.second - v.second, 2));
        double vd = sqrt(pow(v.first - d.first, 2) + pow(v.second - d.second, 2));
        if (ov + vd <= od + lamb) {
            if (mOriginalGraph.getNode(n).isBorderNode()) {
                continue;
            }
            int targetCluster = mOriginalGraph.getNode(n).getClusterBitmask();
            // Get cluster id from the bitmask
            targetCluster = mBitmaskToCluster.at(targetCluster);
            if (std::find(clusterSequence.begin(), clusterSequence.end(), targetCluster) == clusterSequence.end()) {
                extendableClusters.insert(targetCluster);
            }
        }
    }

    return extendableClusters;
}

/**
 *
 * @param fromCluster Cluster ID of the sender
 * @param status MPI status
 * @return Vehicle ID of the vehicle that could not be routed
 *
 * Receive function to notify that no viable route was found
 */
std::tuple<int, std::vector<int>> Router::receiveRouteNotFound(int fromCluster, const MPI_Status &status) const {
    int size;
    MPI_Get_count(&status, MPI_INT, &size);
    std::vector<int> receivedData(size);
    MPI_Recv(receivedData.data(), size, MPI_INT, fromCluster + 1, static_cast<int>(MessageTags::routeNotFound),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    int vehID = receivedData[0];
    int seqSize = receivedData[1];
    std::vector<int> clusterSequence(receivedData.begin() + 2, receivedData.begin() + 2 + seqSize);
    logMessage("RECV|routeNotFound|from=" + std::to_string(fromCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence));
    return std::make_tuple(vehID, clusterSequence);
}

/**
 *
 * @param toCluster Cluster ID of the receiver
 * @param vehID Vehicle ID of the vehicle that could not be routed
 *
 * Send function to notify that no viable route was found
 */
void Router::sendRouteNotFound(int toCluster, int vehID, std::vector<int> clusterSequence) const {
    logMessage("SEND|routeNotFound|to=" + std::to_string(toCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence));
    std::vector<int> sendData;
    sendData.push_back(vehID);
    sendData.push_back(clusterSequence.size());
    sendData.insert(sendData.end(), clusterSequence.begin(), clusterSequence.end());
    MPI_Bsend(sendData.data(), static_cast<int>(sendData.size()), MPI_INT, toCluster + 1,
              static_cast<int>(MessageTags::routeNotFound), MPI_COMM_WORLD);
}

/**
 *
 * @param toCluster Cluster ID of the receiver
 * @param vehID Vehicle ID of the vehicle to vbe routed
 * @param goalEdge Goal edge of the vehicle
 * @param priorities Priorities of the vehicle
 * @param openLabels Open labels of the current HierMLS query for the vehicle route
 *
 * Send function to send the query extension request and border labels to extend the route
 */
void Router::sendContinueRoute(
    int toCluster, int vehID, std::vector<int> clusterSequence, int goalEdge,
    const std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
        &openLabels) {
    logMessage("SEND|continueRoute|to=" + std::to_string(toCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|goalEdge=" + std::to_string(goalEdge) +
               "|openLabels=" + std::to_string(openLabels.size()));
    std::vector<double> combinedData;
    combinedData.push_back(static_cast<double>(vehID));
    combinedData.push_back(static_cast<double>(clusterSequence.size()));
    combinedData.insert(combinedData.end(), clusterSequence.begin(), clusterSequence.end());
    combinedData.push_back(static_cast<double>(goalEdge));

    int labelSize = NUM_PARAMETERS + 3;

    // numParameters, numnodes, node, numedges, edge, numlabels, labels ..., edge, numlabels, labels..., node, numedges,
    // edge, numlabels, labels ..., .......
    combinedData.push_back(static_cast<double>(openLabels.size()));
    for (auto &[node, p] : openLabels) {
        combinedData.push_back(static_cast<double>(node));
        combinedData.push_back(static_cast<double>(p.size()));
        for (auto &[edge, labelSet] : p) {
            combinedData.push_back(static_cast<double>(edge));
            combinedData.push_back(static_cast<double>(labelSet.size()));
            combinedData.reserve(combinedData.size() + (labelSet.size() * labelSize));
            for (auto &l : labelSet) {
                combinedData.push_back(static_cast<double>(l->node));
                combinedData.insert(combinedData.end(), l->parameters.begin(), l->parameters.end());
                combinedData.push_back(static_cast<double>(l->usedEdge->getId()));
                combinedData.push_back(static_cast<double>(l->usedEdge->getInsideEdges().size()));
                combinedData.insert(combinedData.end(), l->usedEdge->getInsideEdges().begin(),
                                    l->usedEdge->getInsideEdges().end());
                combinedData.push_back(static_cast<double>(l->numHops));
            }
        }
    }

    MPI_Bsend(combinedData.data(), static_cast<int>(combinedData.size()), MPI_DOUBLE, toCluster + 1,
              static_cast<int>(MessageTags::continueRoute), MPI_COMM_WORLD);
}

/**
 *
 * @param fromCluster Cluster ID of the sender
 * @param status MPI status
 * @return Vehicle ID, goal edge, priorities and open labels of the current HierMLS query for the vehicle route to be
 * extended in own cluster
 *
 * Receive function to receive the query extension request and border labels to extend the route
 */
std::tuple<int, std::vector<int>, int,
           std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
Router::receiveContinueRoute(int fromCluster, const MPI_Status &status) {
    int size;
    MPI_Get_count(&status, MPI_DOUBLE, &size);
    std::vector<double> receivedData(size);
    MPI_Recv(receivedData.data(), size, MPI_DOUBLE, fromCluster + 1, static_cast<int>(MessageTags::continueRoute),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int vehID = static_cast<int>(receivedData[0]);
    int seqSize = static_cast<int>(receivedData[1]);
    std::vector<int> clusterSequence(receivedData.begin() + 2, receivedData.begin() + 2 + seqSize);
    int goalEdge = static_cast<int>(receivedData[2 + seqSize]);

    logMessage("RECV|continueRoute|from=" + std::to_string(fromCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|goalEdge=" + std::to_string(goalEdge));

    int currentIndex = seqSize + 3;

    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> labels;

    // Decode the vector
    int mapSize = static_cast<int>(receivedData[currentIndex++]);
    for (int s = 0; s < mapSize; ++s) {
        int currentNode = static_cast<int>(receivedData[currentIndex++]);

        int numEdges = static_cast<int>(receivedData[currentIndex++]);
        for (int e = 0; e < numEdges; ++e) {
            int currentEdge = static_cast<int>(receivedData[currentIndex++]);

            int numLabels = static_cast<int>(receivedData[currentIndex++]);
            for (int l = 0; l < numLabels; ++l) {
                int node = static_cast<int>(receivedData[currentIndex++]);

                std::vector<double> parameters;
                parameters.reserve(NUM_PARAMETERS);
                for (int i = 0; i < NUM_PARAMETERS; ++i) {
                    parameters.push_back(receivedData[currentIndex++]);
                }

                int usedEdgeId = static_cast<int>(receivedData[currentIndex++]);

                int numInsideEdges = static_cast<int>(receivedData[currentIndex++]);
                std::vector<int> insideEdges;
                insideEdges.reserve(numInsideEdges);
                for (int i = 0; i < numInsideEdges; ++i) {
                    insideEdges.push_back(static_cast<int>(receivedData[currentIndex++]));
                }

                int numHops = static_cast<int>(receivedData[currentIndex++]);

                // CoverEdge cv(usedEdgeId, sourceCluster);
                labels[currentNode][currentEdge].insert(std::make_shared<hierLabel>(
                    node, parameters, std::make_shared<CoverEdge>(usedEdgeId, fromCluster, insideEdges), numHops));
            }
        }
    }

    return std::make_tuple(vehID, clusterSequence, goalEdge, labels);
}

// sendFoundRoutes(sourceCluster, vehID, paths);
void Router::sendFoundRoutes(int toCluster, int vehID, std::vector<int> clusterSequence,
                             const std::vector<std::vector<double>> &routes) {
    logMessage("SEND|foundRoutes|to=" + std::to_string(toCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|routes=" + std::to_string(routes.size()));
    std::vector<double> sendData;
    sendData.push_back(static_cast<double>(vehID));
    sendData.push_back(static_cast<double>(clusterSequence.size()));
    sendData.insert(sendData.end(), clusterSequence.begin(), clusterSequence.end());
    sendData.push_back(static_cast<double>(routes.size()));
    for (const auto &route : routes) {
        sendData.insert(sendData.end(), route.begin(), route.end());
    }
    MPI_Bsend(sendData.data(), static_cast<int>(sendData.size()), MPI_DOUBLE, toCluster + 1,
              static_cast<int>(MessageTags::foundRoutes), MPI_COMM_WORLD);
}

// auto [vehID, foundPaths] = receiveFoundRoutes(n, status);
std::tuple<int, std::vector<int>, std::vector<std::vector<double>>> Router::receiveFoundRoutes(
    int fromCluster, const MPI_Status &status) {
    int size;
    MPI_Get_count(&status, MPI_DOUBLE, &size);
    std::vector<double> receivedData(size);
    MPI_Recv(receivedData.data(), size, MPI_DOUBLE, fromCluster + 1, static_cast<int>(MessageTags::foundRoutes),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int vehID = static_cast<int>(receivedData[0]);
    int seqSize = static_cast<int>(receivedData[1]);
    std::vector<int> clusterSequence(receivedData.begin() + 2, receivedData.begin() + 2 + seqSize);
    int numRoutes = static_cast<int>(receivedData[2 + seqSize]);

    logMessage("RECV|foundRoutes|from=" + std::to_string(fromCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|numRoutes=" + std::to_string(numRoutes));

    std::vector<std::vector<double>> routes;
    for (int i = seqSize + 3; i < numRoutes + seqSize + 3; i += NUM_PARAMETERS) {
        std::vector<double> route(receivedData.begin() + i, receivedData.begin() + i + NUM_PARAMETERS);
        routes.push_back(route);
    }

    return std::make_tuple(vehID, clusterSequence, routes);
}

// sendAcceptedRoute(targetCluster, vehID, storedPriorities[vehID], bestParameters);
void Router::sendAcceptedRoute(int toCluster, int vehID, const std::vector<double> &priorities,
                               const std::vector<double> &route) const {
    logMessage("SEND|acceptedRoute|to=" + std::to_string(toCluster) + "|veh=" + std::to_string(vehID) +
               "|priorities=" + vectorToString(priorities) + "|route=" + vectorToString(route));
    std::vector<double> sendData;
    sendData.push_back(static_cast<double>(vehID));
    sendData.insert(sendData.end(), priorities.begin(), priorities.end());
    sendData.insert(sendData.end(), route.begin(), route.end());
    MPI_Bsend(sendData.data(), static_cast<int>(sendData.size()), MPI_DOUBLE, toCluster + 1,
              static_cast<int>(MessageTags::acceptedRoute), MPI_COMM_WORLD);
}
// auto [vehID, priorities, acceptedRoute] = receiveAcceptedRoute(n, status);
/** @brief Receive message that requested route could not be found */
std::tuple<int, std::vector<double>, std::vector<double>> Router::receiveAcceptedRoute(int fromCluster,
                                                                                       const MPI_Status &status) const {
    int size;
    MPI_Get_count(&status, MPI_DOUBLE, &size);
    std::vector<double> receivedData(size);
    MPI_Recv(receivedData.data(), size, MPI_DOUBLE, fromCluster + 1, static_cast<int>(MessageTags::acceptedRoute),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int vehID = static_cast<int>(receivedData[0]);
    std::vector<double> priorities(receivedData.begin() + 1, receivedData.begin() + 1 + NUM_PARAMETERS);
    std::vector<double> route(receivedData.begin() + 1 + NUM_PARAMETERS, receivedData.end());

    logMessage("RECV|acceptedRoute|from=" + std::to_string(fromCluster) + "|veh=" + std::to_string(vehID) +
               "|priorities=" + vectorToString(priorities) + "|route=" + vectorToString(route));

    return std::make_tuple(vehID, priorities, route);
}

/**
 *
 * @param fromCluster Cluster ID of the sender
 * @param status MPI status
 * @return Vehicle ID, built portion of the path and the border label of the route in own cluster
 *
 * Receive function to receive the completed route.
 * Receives partial border label used to find own label in memory and use it to continue building the route
 */
std::tuple<int, std::vector<int>, std::vector<int>, std::vector<double>, std::shared_ptr<hierLabel>>
Router::receiveFinishedRoute(int fromCluster, const MPI_Status &status) const {
    int size;
    MPI_Get_count(&status, MPI_DOUBLE, &size);
    std::vector<double> receivedData(size);
    MPI_Recv(receivedData.data(), size, MPI_DOUBLE, fromCluster + 1, static_cast<int>(MessageTags::finishedRoute),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int vehID = static_cast<int>(receivedData[0]);
    int seqSize = static_cast<int>(receivedData[1]);
    std::vector<int> clusterSequence(receivedData.begin() + 2, receivedData.begin() + 2 + seqSize);
    int buildPathSize = static_cast<int>(receivedData[seqSize + 2]);
    int newPos = seqSize + 3;
    std::vector<int> buildPath(receivedData.begin() + newPos, receivedData.begin() + newPos + buildPathSize);
    std::vector<double> priorities(receivedData.begin() + newPos + buildPathSize,
                                   receivedData.begin() + newPos + buildPathSize + NUM_PARAMETERS);

    // Read label
    int pos = newPos + buildPathSize + NUM_PARAMETERS;
    int node = static_cast<int>(receivedData[pos++]);
    int numParams = static_cast<int>(receivedData[pos++]);
    std::vector<double> params(receivedData.begin() + pos, receivedData.begin() + pos + numParams);
    pos += numParams;
    int edgeID = static_cast<int>(receivedData[pos++]);
    int numInsideEdges = static_cast<int>(receivedData[pos++]);
    std::vector<int> insideEdges(receivedData.begin() + pos, receivedData.begin() + pos + numInsideEdges);
    pos += numInsideEdges;
    int numHops = static_cast<int>(receivedData[pos]);

    logMessage("RECV|finishedRoute|from=" + std::to_string(fromCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|buildPath=" + vectorToString(buildPath) +
               "|priorities=" + vectorToString(priorities) + "|label={node=" + std::to_string(node) +
               "|params=" + vectorToString(params) + "|edgeID=" + std::to_string(edgeID) +
               "|insideEdges=" + vectorToString(insideEdges) + "|numHops=" + std::to_string(numHops) + "}");

    std::shared_ptr<hierLabel> receivedLabel = std::make_shared<hierLabel>(
        node, params, std::make_shared<CoverEdge>(edgeID, fromCluster, insideEdges), numHops);

    return std::make_tuple(vehID, clusterSequence, buildPath, priorities, receivedLabel);
}

/**
 *
 * @param toCluster Cluster ID of the receiver
 * @param vehID Vehicle ID of the vehicle to be routed
 * @param buildPath Portion of the route built in own and further clusters
 * @param rootLabel Border label of the route in own cluster
 *
 * Send function to send the completed route.
 * Sends the border label used to find own label in memory and use it to continue building the route
 */
void Router::sendFinishedRoute(int toCluster, int vehID, std::vector<int> clusterSequence,
                               const std::vector<int> &buildPath, const std::vector<double> &priorities,
                               const std::shared_ptr<hierLabel> &rootLabel) const {
    logMessage("SEND|finishedRoute|to=" + std::to_string(toCluster) + "|veh=" + std::to_string(vehID) +
               "|clusterSequence=" + vectorToString(clusterSequence) + "|buildPath=" + vectorToString(buildPath) +
               "|priorities=" + vectorToString(priorities));
    std::vector<double> sendData;
    sendData.push_back(static_cast<double>(vehID));
    sendData.push_back(static_cast<double>(clusterSequence.size()));
    sendData.insert(sendData.end(), clusterSequence.begin(), clusterSequence.end());
    sendData.push_back(static_cast<double>(buildPath.size()));
    sendData.insert(sendData.end(), buildPath.begin(), buildPath.end());
    sendData.insert(sendData.end(), priorities.begin(), priorities.end());
    sendData.push_back(static_cast<double>(rootLabel->node));
    sendData.push_back(static_cast<double>(rootLabel->parameters.size()));
    sendData.insert(sendData.end(), rootLabel->parameters.begin(), rootLabel->parameters.end());
    sendData.push_back(static_cast<double>(rootLabel->usedEdge->getId()));
    sendData.push_back(static_cast<double>(rootLabel->usedEdge->getInsideEdges().size()));
    sendData.insert(sendData.end(), rootLabel->usedEdge->getInsideEdges().begin(),
                    rootLabel->usedEdge->getInsideEdges().end());
    sendData.push_back(static_cast<double>(rootLabel->numHops));

    MPI_Bsend(sendData.data(), static_cast<int>(sendData.size()), MPI_DOUBLE, toCluster + 1,
              static_cast<int>(MessageTags::finishedRoute), MPI_COMM_WORLD);
}

/**
 *
 * @param isActive True if the process is active, false otherwise
 *
 * Send function to notify the master process about the activity status of the ACU
 */
void Router::sendActivityStatus(bool isActive) {
    logMessage("SEND|activityStatus|to=MASTER|status=" + std::string(isActive ? "active" : "finished"));
    int dump = 0;
    if (isActive) {
        MPI_Bsend(&dump, 1, MPI_INT, 0, static_cast<int>(MessageTags::processActive), MPI_COMM_WORLD);
    } else {
        MPI_Bsend(&dump, 1, MPI_INT, 0, static_cast<int>(MessageTags::processFinished), MPI_COMM_WORLD);
    }
}

/**
 *
 * @param edge Cover edge to predict parameters for
 * @param enterTime Time of entering the coverf edge
 * @return Tuple of predicted average density, traversal time and fuel consumption
 *
 * Combined predictor function for dynamic parameters of the cover edge.
 * Processes inner edges of the cover edge one-by-one and returns total values.
 * Total density is averaged over all inner edges.
 * Total time is computed as a sum of traversal times over all inner edges.
 * Total fuel consumption is computed as a sum of fuel consumptions over all inner edges.
 */
std::tuple<double, double, double> Router::predictDynamicParameters(const CoverEdge &edge, int enterTime) {
    const auto &insideEdges = edge.getInsideEdges();
    if (insideEdges.empty()) {
        return std::make_tuple(0.0, 0.0, 0.0);
    }

    double totalDensity = 0;
    double totalTime = 0;
    double totalFuel = 0;
    double currentTime = enterTime;

    for (int e : insideEdges) {
        const Edge &currentEdge = mOriginalGraph.getEdge(e);
        const double edgeLength = currentEdge.getLength();
        const int interval = timeToInterval(currentTime);
        const std::pair<int, int> &flows = mEdgeFlows[e][interval];
        const double recordedDensity = mEdgeDensities[e][interval];

        const double currentDensity = mTrafficEstimator.predictDensity(currentEdge, flows, recordedDensity);
        const double currentSpeed = mSpeedEstimator.predictSpeed(currentEdge, currentDensity);
        const double currentFuel = mFuelEstimator.predictFuel(currentEdge, currentDensity, currentSpeed);

        totalDensity += currentDensity;
        totalTime += edgeLength / currentSpeed;
        totalFuel += currentFuel;
    }

    return std::make_tuple(totalDensity / insideEdges.size(), totalTime, totalFuel);
}

/**
 *
 * @param edge Original-graph edge to predict parameters for
 * @param enterTime Time of entering the coverf edge
 * @return Tuple of predicted average density, speed and fuel consumption
 *
 * Combined predictor function for dynamic parameters of the original-graph edge.
 * Uses flows and previously predicted densities to make new predictions.
 * Predicts density, speed and fuel consumption.
 */
std::tuple<double, double, double> Router::predictDynamicParameters(const Edge &edge, const int enterTime) {
    const int interval = timeToInterval(enterTime);
    if (interval < 0) {
        return std::make_tuple(0.0, edge.getSpeedLimit(), 0.0);
    }

    // If current interval is requested, provide current data
    if (interval == 0) {
        return std::make_tuple(mEdgeDensities[edge.getID()][0], mEdgeSpeeds[edge.getID()][0],
                               mEdgeFuels[edge.getID()][0]);
    }

    // Ensure vectors have enough capacity
    auto &flows = mEdgeFlows[edge.getID()];
    auto &densities = mEdgeDensities[edge.getID()];
    auto &speeds = mEdgeSpeeds[edge.getID()];
    auto &fuels = mEdgeFuels[edge.getID()];

    if (flows.size() <= interval) {
        return std::make_tuple(0.0, edge.getSpeedLimit(), mFuelEstimator.predictFuel(edge, 0, edge.getSpeedLimit()));
    } else {
        const double recordedDensity = densities[interval - 1];
        const std::pair<int, int> &currentFlows = flows[interval];

        const double currentDensity = mTrafficEstimator.predictDensity(edge, currentFlows, recordedDensity);
        const double currentSpeed = mSpeedEstimator.predictSpeed(edge, currentDensity);
        const double currentFuel = mFuelEstimator.predictFuel(edge, currentDensity, currentSpeed);

        return std::make_tuple(currentDensity, currentSpeed, currentFuel);
    }

    // if (flows.size() < interval + 1) {
    //     flows.resize(interval + 2, {0, 0});
    // }
    // if (densities.size() < interval + 1) {
    //     densities.resize(interval + 2, 0);
    //     speeds.resize(interval + 2, edge.getSpeedLimit());
    //     fuels.resize(interval + 2, mFuelEstimator.predictFuel(edge, 0, edge.getSpeedLimit()));
    // }
}

/**
 *
 * @param edge Cover edge to predict parameters for
 * @param enterTime Time of entering the cover edge
 * @return Vector of predicted parameters (distance, time, fuel, density)
 *
 * Prepares planning parameters for a newly-visited edge.
 * Distance is static parameter, the rest are dynamic and predicted
 */
std::vector<double> Router::prepareDynamicParameters(const CoverEdge &edge, double enterTime) {
    if (edge.getInsideEdges().empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    const auto [density, time, fuel] = predictDynamicParameters(edge, enterTime);
    return {edge.getLength(), time, fuel, density};
}

std::vector<double> Router::extractDynamicParameters(const Edge &edge, double enterTime) {
    int currentInterval = timeToInterval(enterTime);
    int e = edge.getID();

    // Check the values are present, create them if not
    while (mEdgeDensities[e].size() <= currentInterval + 1) {
        mEdgeDensities[e].push_back(0);
        mEdgeSpeeds[e].push_back(mOriginalGraph.getEdge(e).getSpeedLimit());
        mEdgeFuels[e].push_back(
            mFuelEstimator.predictFuel(mOriginalGraph.getEdge(e), 0, mOriginalGraph.getEdge(e).getSpeedLimit()));
    }

    double currentDensity = mEdgeDensities[e][currentInterval];
    double currentSpeed = mEdgeSpeeds[e][currentInterval];
    double currentFuel = mEdgeFuels[e][currentInterval];
    // double currentDanger = edge.getDangerFactor();
    double currentNumHops = 1;

    // return {mOriginalGraph.getEdge(e).getLength(), mOriginalGraph.getEdge(e).getLength() /
    // mEdgeSpeeds[e][currentInterval], mEdgeFuels[e][currentInterval], mEdgeDensities[e][currentInterval]};
    return {mOriginalGraph.getEdge(e).getLength(),
            mOriginalGraph.getEdge(e).getLength() / mEdgeSpeeds[e][currentInterval], mEdgeFuels[e][currentInterval],
            currentNumHops};
}

/**
 *
 * @param vehID Vehicle ID of the vehicle to be routed
 * @param route Route of the vehicle to be recorded
 * @param times Traversal times of the edge in the route
 * @param priorities Priorities of the vehicle
 * @param currentTime Route start time
 *
 * Records the provided route of the vehicle and the rest of the parameters.
 * Updates inner predictions of dynamic parameters based on the update flows from the new vehicle
 */
void Router::recordRoute(int vehID, const std::vector<int> &route, std::vector<double> times,
                         const std::vector<double> &priorities, int currentTime) {

    // Edges in the route are original edges
    int startIndex;
    if (times.empty()) {
        startIndex = 0;
    } else {
        startIndex = 1;
    }

    for (int e = startIndex; e < route.size(); ++e) {
        int i = route[e];

        if (!mOriginalGraph.getEdge(i).inCluster(mClusterID)) {
            double traversalTime =
                mOriginalGraph.getEdge(i).getLength() / (mOriginalGraph.getEdge(i).getSpeedLimit() / 2);
            currentTime += static_cast<int>(traversalTime);
        } else {
            // Compute inflow interval by rounding down
            int startInterval = timeToInterval(currentTime);
            // Predict new dynamic parameters using predictParameters
            auto [density, speed, fuel] = predictDynamicParameters(mOriginalGraph.getEdge(i), currentTime);
            double traversalTime = mOriginalGraph.getEdge(i).getLength() / speed;
            int endInterval = timeToInterval(currentTime + traversalTime);

            while (mEdgeFlows[i].size() <= endInterval + 1) {
                mEdgeFlows[i].emplace_back(0, 0);
            }
            while (mEdgeDensities[i].size() <= endInterval + 1) {
                mEdgeDensities[i].push_back(0);
                mEdgeSpeeds[i].push_back(mOriginalGraph.getEdge(i).getSpeedLimit());
                mEdgeFuels[i].push_back(mFuelEstimator.predictFuel(mOriginalGraph.getEdge(i), 0,
                                                                   mOriginalGraph.getEdge(i).getSpeedLimit()));
            }

            mEdgeFlows[i][startInterval].first += 1;
            mEdgeFlows[i][endInterval].second += 1;

            mEdgeDensities[i][startInterval] = density;
            mEdgeSpeeds[i][startInterval] = speed;
            mEdgeFuels[i][startInterval] = fuel;

            int tempTime = currentTime + mIntervalLength;
            for (int t = startInterval + 1; t < mEdgeDensities[i].size(); ++t) {
                auto [tempDensity, tempSpeed, tempFuel] = predictDynamicParameters(mOriginalGraph.getEdge(i), tempTime);

                if (fabs(mEdgeDensities[i][t] - tempDensity) <= 0.01) {
                    break;
                }

                mEdgeDensities[i][t] = tempDensity;
                mEdgeSpeeds[i][t] = tempSpeed;
                mEdgeFuels[i][t] = tempFuel;

                tempTime += mIntervalLength;
            }

            currentTime += traversalTime;
        }
        times.push_back(currentTime);
    }
    // Cut off the rest of the route
    std::vector<int> newRoute(route.begin(), route.begin() + times.size());
    mVehicleTimes[vehID] = times;
    mVehicleRoutes[vehID] = newRoute;
    mVehiclePriorities[vehID] = priorities;

    logMessage("ROUTE|record|veh=" + std::to_string(vehID) + "|route=" + vectorToString(route) +
               "|times=" + vectorToString(times) + "|priorities=" + vectorToString(priorities));
}

/**
 *
 * @param originalLabel Original label to build the path from
 * @param path Vector to store the path as list of edges
 * @return Route label of the path
 *
 * Builds the path from the original label from back to front and returns the start label.
 * The start label can then be sent with the route to the parent cluster to continue building the preceding part of
 * the route
 */
std::shared_ptr<hierLabel> Router::buildForwardPath(const std::shared_ptr<hierLabel> &originalLabel,
                                                    std::vector<int> &path) {
    auto currentLabel = originalLabel;
    while (currentLabel->pParent != nullptr) {
        std::vector<int> insideEdges = currentLabel->usedEdge->getInsideEdges();
        path.insert(path.begin(), insideEdges.begin(), insideEdges.end());
        currentLabel = currentLabel->pParent;
    }
    return currentLabel;
}

/**
 *
 * @param labels Map of labels to be sent
 * @param destinationCluster Cluster ID of the receiver
 *
 * Send function to send the labels to the parent cluster. Used in continueRoute requests
 */
void Router::sendLabels(
    const std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
        &labels,
    int destinationCluster) {
    int destinationAddress = destinationCluster + 1;

    std::vector<double> sendingVector;
    int numParameters = static_cast<int>(labels.begin()->second.begin()->second.begin()->get()->parameters.size());
    int labelSize = numParameters + 3;

    // numParameters, numnodes, node, numedges, edge, numlabels, labels ..., edge, numlabels, labels..., node, numedges,
    // edge, numlabels, labels ..., .......
    sendingVector.push_back(static_cast<double>(numParameters));
    sendingVector.push_back(static_cast<double>(labels.size()));
    for (auto &[node, p] : labels) {
        sendingVector.push_back(static_cast<double>(node));
        sendingVector.push_back(static_cast<double>(p.size()));
        for (auto &[edge, q] : p) {
            sendingVector.push_back(static_cast<double>(edge));
            sendingVector.push_back(static_cast<double>(q.size()));
            for (auto &label : q) {
                sendingVector.push_back(static_cast<double>(label->node));
                for (auto &parameter : label->parameters) {
                    sendingVector.push_back(parameter);
                }
                sendingVector.push_back(static_cast<double>(label->usedEdge->getId()));
                sendingVector.push_back(static_cast<double>(label->usedEdge->getInsideEdges().size()));
                for (auto &insideEdge : label->usedEdge->getInsideEdges()) {
                    sendingVector.push_back(static_cast<double>(insideEdge));
                }
                sendingVector.push_back(static_cast<double>(label->numHops));
            }
        }
    }

    logMessage("SEND|labels|to=" + std::to_string(destinationCluster) + "|count=" + std::to_string(labels.size()) +
               "|data=" + vectorToString(sendingVector));

    MPI_Bsend(sendingVector.data(), static_cast<int>(sendingVector.size()), MPI_DOUBLE, destinationAddress,
              static_cast<int>(MessageTags::label), MPI_COMM_WORLD);
}

/**
 *
 * @param sourceCluster Cluster ID of the sender
 * @return Map of labels received from the parent cluster
 *
 * Receive function to receive the labels from the parent cluster. Used in continueRoute requests
 */
std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
Router::receiveLabels(int sourceCluster) {
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> labels;
    int sourceAddress = sourceCluster + 1;

    std::vector<double> receivingVector;
    MPI_Status status;
    int count;
    MPI_Probe(sourceAddress, static_cast<int>(MessageTags::continueRoute), MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_DOUBLE, &count);
    receivingVector.resize(count);
    MPI_Recv(receivingVector.data(), count, MPI_DOUBLE, sourceAddress, static_cast<int>(MessageTags::continueRoute),
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    // Decode the vector
    int numParameters = static_cast<int>(receivingVector[0]);
    int mapSize = static_cast<int>(receivingVector[1]);
    int currentIndex = 2;
    for (int s = 0; s < mapSize; ++s) {
        int currentNode = static_cast<int>(receivingVector[currentIndex++]);

        int numEdges = static_cast<int>(receivingVector[currentIndex++]);
        for (int e = 0; e < numEdges; ++e) {
            int currentEdge = static_cast<int>(receivingVector[currentIndex++]);

            int numLabels = static_cast<int>(receivingVector[currentIndex++]);
            for (int l = 0; l < numLabels; ++l) {
                int node = static_cast<int>(receivingVector[currentIndex++]);

                std::vector<double> parameters;
                parameters.reserve(numParameters);
                for (int i = 0; i < numParameters; ++i) {
                    parameters.push_back(receivingVector[currentIndex++]);
                }

                int usedEdgeId = static_cast<int>(receivingVector[currentIndex++]);

                int numInsideEdges = static_cast<int>(receivingVector[currentIndex++]);
                std::vector<int> insideEdges;
                insideEdges.reserve(numInsideEdges);
                for (int i = 0; i < numInsideEdges; ++i) {
                    insideEdges.push_back(static_cast<int>(receivingVector[currentIndex++]));
                }

                int numHops = static_cast<int>(receivingVector[currentIndex++]);

                // CoverEdge cv(usedEdgeId, sourceCluster);
                labels[currentNode][currentEdge].insert(std::make_shared<hierLabel>(
                    node, parameters, std::make_shared<CoverEdge>(usedEdgeId, sourceCluster, insideEdges), numHops));
            }
        }
    }

    return labels;
}

/**
 *
 * @param startEdge Starting edge of the route
 * @param goalEdge Goal edge of the route
 * @param sourceCluster ID of the cluster the routing request originated from
 * @param openLabels Open labels of a query that originated elsewhere
 * @return Map of border labels to be forwarded to the next cluster for further routing
 *
 * This is a Hierarchical MLS function used when the goal edge is not in the current cluster.
 * The aim of the function is to extend the route (from start or received open labels) to the borders of the cluster.
 * The labels from the borders are then returned to be forwarded to other clusters
 */
std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
Router::distributedHierarchicalSearch(
    int startEdge, int goalEdge, std::unordered_set<int> &extendableClusters,
    std::optional<
        std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
        openLabels) {
    // The function is used when it is evident that the goal edge is in a different cluster
    // The query progresses until the edges of the area
    // Returns the open labels, closed labels, and t sets of the border vertices

    auto start = std::chrono::high_resolution_clock::now();

    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>
        labelsToForward;

    if (!openLabels.has_value()) {
        // Initialize the optional in place if it's empty
        openLabels.emplace();
    }

    // Use forward MLS
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> closedLabels;
    std::unordered_map<int, std::unordered_map<int, std::set<std::vector<double>>>> tSets;

    std::set<std::shared_ptr<hierLabel>, hierLexCompare> toExpand;

    for (auto &[node, p] : (*openLabels)) {
        for (auto &[ed, labels] : p) {
            toExpand.insert(labels.begin(), labels.end());
        }
    }

    if (toExpand.empty()) {
        int startNode = mOriginalGraph.getEdge(startEdge).getEndNode();
        const CoverEdge &startCoverEdge = mCoverGraphs[0]->originalToCoverEdge(startEdge);
        std::vector<double> startParameters = extractDynamicParameters(mOriginalGraph.getEdge(startEdge), mCurrentTime);

        auto pStartLabel = std::make_shared<hierLabel>(startNode, startParameters, nullptr,
                                                       std::make_shared<CoverEdge>(startCoverEdge), 1);
        toExpand.insert(pStartLabel);
        (*openLabels)[startNode][startEdge].insert(pStartLabel);
    }

    int iters = 0;
    while (!toExpand.empty()) {
        auto pCurrentLabel = *toExpand.begin();
        toExpand.erase(toExpand.begin());

        int currentNode = pCurrentLabel->node;

        int currentLevel = std::min(mNodeLevels[currentNode], mLevelLimit);
        std::vector<double> currentParameters = pCurrentLabel->parameters;
        int incomingEdge = pCurrentLabel->usedEdge->getEndEdge();

        iters += 1;

        (*openLabels)[currentNode][incomingEdge].erase(pCurrentLabel);
        closedLabels[currentNode][incomingEdge].insert(pCurrentLabel);
        updateTSet(tSets[currentNode][incomingEdge], pCurrentLabel);

        // Expand to neighbor vertices
        for (int edge : mCoverGraphs[currentLevel]->getConnectedOutgoingEdges(incomingEdge)) {
            const CoverEdge &rNextEdge = mCoverGraphs[currentLevel]->getCoverEdge(edge);

            int nextNode = rNextEdge.getEndNode();

            // Check for twice visited simple edges
            std::unordered_set<int> visitedEdges;
            bool twiceVisited = false;
            auto tempLabel = pCurrentLabel;
            while (tempLabel->pParent != nullptr) {
                for (int edge : tempLabel->usedEdge->getInsideEdges()) {
                    if (visitedEdges.count(edge) > 0) {
                        twiceVisited = true;
                        break;
                    }
                    visitedEdges.insert(edge);
                }

                tempLabel = tempLabel->pParent;
            }

            if (twiceVisited) {
                continue;
            }

            // if (nextNode == currentNode || (pCurrentLabel->pParent && nextNode == pCurrentLabel->pParent->node)) {
            //     continue;
            // }

            if (!mOriginalGraph.getNode(nextNode).inCluster(mClusterID)) {
                continue;
            }

            bool toForward = false;
            // Check if the edge is a border edge
            if (mOriginalGraph.getNode(currentNode).isBorderNode() && mOriginalGraph.getNode(nextNode).isBorderNode() &&
                rNextEdge.getInsideEdges().size() == 1 &&
                mOriginalGraph.getEdge(rNextEdge.getInsideEdges()[0]).isBorderEdge()) {
                // Check if the source cluster id is not the same as the sourceCluster
                int nextClusterBitmask = mOriginalGraph.getEdge(rNextEdge.getInsideEdges()[0]).getClusterBitmask();
                nextClusterBitmask = nextClusterBitmask & ~(1 << mClusterID);
                int otherCluster = mBitmaskToCluster[nextClusterBitmask];
                if (extendableClusters.count(otherCluster) > 0) {
                    toForward = true;
                }
            }

            bool congested = false;
            // Prepare planning parameters for the next edge
            for (int e : rNextEdge.getInsideEdges()) {
                if (mOriginalGraph.getEdge(e).getLength() < 15) {
                    continue;
                }
                // Get the density of the edge if interval exists
                int currentInterval = timeToInterval(mCurrentTime + pCurrentLabel->parameters[1]);
                double dens = 0;
                if (currentInterval < mEdgeDensities[e].size()) {
                    dens = mEdgeDensities[e][currentInterval];
                }

                if (dens > 0.95) {
                    congested = true;
                    break;
                }
            }
            if (congested) {
                continue;
            }

            // Prepare planning parameters for the next edge
            std::shared_ptr<hierLabel> nextLabel = expandLabel(pCurrentLabel, std::make_shared<CoverEdge>(rNextEdge));
            std::vector<double> nextParameters = nextLabel->parameters;

            int nextIncomingEdge = rNextEdge.getEndEdge();

            if (toForward) {
                // Check against existing labels
                bool isDominated = false;
                std::stack<std::shared_ptr<hierLabel>> toRemove;
                for (auto &rLab : labelsToForward[nextNode][nextIncomingEdge]) {
                    if (checkDomination(rLab->parameters, nextParameters)) {
                        isDominated = true;
                        break;
                    }
                    if (checkDomination(nextParameters, rLab->parameters)) {
                        toRemove.push(rLab);
                    }
                }
                if (isDominated)
                    continue;
                while (!toRemove.empty()) {
                    auto pLab = toRemove.top();
                    toRemove.pop();
                    labelsToForward[nextNode][nextIncomingEdge].erase(pLab);
                    toExpand.erase(pLab);
                }
                labelsToForward[nextNode][nextIncomingEdge].insert(nextLabel);
                continue;
            }

            if (tDiscard(tSets[nextNode][nextIncomingEdge], nextParameters))
                continue;

            bool isDominated = false;
            std::stack<std::shared_ptr<hierLabel>> toRemove;
            for (auto &rLab : (*openLabels)[nextNode][nextIncomingEdge]) {
                if (checkDomination(rLab->parameters, nextParameters)) {
                    isDominated = true;
                    break;
                }
                if (checkDomination(nextParameters, rLab->parameters)) {
                    toRemove.push(rLab);
                }
            }
            if (isDominated)
                continue;
            while (!toRemove.empty()) {
                auto pLab = toRemove.top();
                toRemove.pop();
                (*openLabels)[nextNode][nextIncomingEdge].erase(pLab);
                toExpand.erase(pLab);
            }

            (*openLabels)[nextNode][nextIncomingEdge].insert(nextLabel);
            toExpand.insert(nextLabel);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    logMessage("Elapsed time for distributed search: " + std::to_string(elapsed.count()) + "s");

    return labelsToForward;
}

/**
 *
 * @param startEdge Starting edge of the route
 * @param goalEdge Goal edge of the route
 * @param openLabels Open labels of a query that originated elsewhere
 * @return Set of labels at the goal representing completed paths
 *
 * This is a Hierarchical MLS function used when the goal edge is in the current cluster.
 * The function backward search to connect the goal edge to the top cover.
 * The result is a set of labels representing completed paths that can be used to build the route.
 */
std::set<std::shared_ptr<hierLabel>, hierLexCompare> Router::completeHierarchicalSearch(
    int startEdge, int goalEdge,
    std::optional<
        std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>>>
        openLabels) {
    // The planning function that expects the goal edge to belong to this cluster
    // As such, this function connects the goal edge to the top cover level and connects further paths to it
    auto start = std::chrono::high_resolution_clock::now();

    if (!openLabels.has_value()) {
        // Initialize the optional in place if it's empty
        openLabels.emplace();
    }

    int goalNode = mOriginalGraph.getEdge(goalEdge).getStartNode();
    // Ensure backward connection from the goal edge
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> hitLabels;
    if (mCoverGraphs[mLevelLimit]->getCoverNodes().count(goalNode) == 0) {
        hitLabels = backwardHierarchicalSearch(goalEdge);
    } else {
        const CoverEdge &goalCoverEdge = mCoverGraphs[0]->originalToCoverEdge(goalEdge);
        auto pGoalLabel =
            std::make_shared<hierLabel>(goalNode, std::vector<double>(NUM_PARAMETERS, 0), nullptr,
                                        std::make_shared<CoverEdge>(goalCoverEdge), goalCoverEdge.getNumHops());
        hitLabels[goalNode][goalEdge].insert(pGoalLabel);
    }

    std::unordered_set<int> topNodes;
    for (const auto &[rNode, rMap] : hitLabels) {
        if (mCoverGraphs[mLevelLimit]->getCoverNodes().count(rNode) > 0) {
            topNodes.insert(rNode);
        }
    }

    // Use forward MLS
    std::set<std::shared_ptr<hierLabel>, hierLexCompare> goalParetoSet;

    std::set<std::shared_ptr<hierLabel>, hierLexCompare> toExpand;
    std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> closedLabels;
    std::unordered_map<int, std::unordered_map<int, std::set<std::vector<double>>>> tSets;

    for (auto &[node, p] : (*openLabels)) {
        for (auto &[ed, labels] : p) {
            toExpand.insert(labels.begin(), labels.end());
        }
    }

    if (toExpand.empty()) {
        int startNode = mOriginalGraph.getEdge(startEdge).getEndNode();
        const CoverEdge &startCoverEdge = mCoverGraphs[0]->originalToCoverEdge(startEdge);
        std::vector<double> startParameters = extractDynamicParameters(mOriginalGraph.getEdge(startEdge), mCurrentTime);

        auto pStartLabel = std::make_shared<hierLabel>(startNode, startParameters, nullptr,
                                                       std::make_shared<CoverEdge>(startCoverEdge), 1);
        toExpand.insert(pStartLabel);
        (*openLabels)[startNode][startEdge].insert(pStartLabel);
    }

    int iters = 0;
    int hitReached = 0;
    while (!toExpand.empty()) {
        auto pCurrentLabel = *toExpand.begin();
        toExpand.erase(toExpand.begin());
        int currentNode = pCurrentLabel->node;
        int currentLevel = std::min(mNodeLevels[currentNode], mLevelLimit);
        std::vector<double> currentParameters = pCurrentLabel->parameters;
        int incomingEdge = pCurrentLabel->usedEdge->getEndEdge();

        iters += 1;

        (*openLabels)[currentNode][incomingEdge].erase(pCurrentLabel);
        closedLabels[currentNode][incomingEdge].insert(pCurrentLabel);
        updateTSet(tSets[currentNode][incomingEdge], pCurrentLabel);

        // Connect to backward paths from goal edge if in a hit vertex
        if (topNodes.count(currentNode) > 0) {
            for (auto &hitPair : hitLabels[currentNode]) {
                // Check connection from the forward incoming edge to backward outgoing
                if (mOriginalGraph.getEdge(incomingEdge).getOutgoingEdges().count(hitPair.first)) {
                    hitReached += 1;
                    for (auto &pHitLabel : hitPair.second) {
                        // For every label from the hit node to the vertex, connect the paths and see if they are in
                        // Pareto set Backward labels do not contain real parameters, hence compute them now Copy
                        // current label
                        auto currentLabel =
                            std::make_shared<hierLabel>(pCurrentLabel->node, currentParameters, pCurrentLabel->pParent,
                                                        pCurrentLabel->usedEdge, pCurrentLabel->numHops);
                        auto currentBackwardLabel = pHitLabel;

                        while (currentBackwardLabel->pParent != nullptr) {
                            currentLabel = expandLabel(currentLabel, currentBackwardLabel->usedEdge);
                            currentBackwardLabel = currentBackwardLabel->pParent;
                        }

                        bool congested = false;
                        // Prepare planning parameters for the next edge
                        for (int e : currentBackwardLabel->usedEdge->getInsideEdges()) {
                            if (mOriginalGraph.getEdge(e).getLength() < 15) {
                                continue;
                            }
                            int currentInterval = timeToInterval(mCurrentTime + pCurrentLabel->parameters[1]);
                            double dens = 0;
                            if (currentInterval < mEdgeDensities[e].size()) {
                                dens = mEdgeDensities[e][currentInterval];
                            }
                            // double dens = extractDynamicParameters(mOriginalGraph.getEdge(e),
                            //                                      mCurrentTime + pCurrentLabel->parameters[1])[3];
                            if (dens > 0.95) {
                                congested = true;
                                break;
                            }
                        }
                        if (congested) {
                            continue;
                        }

                        // Connect goal edge
                        currentLabel = expandLabel(currentLabel, currentBackwardLabel->usedEdge);
                        std::vector<double> newParameters = currentLabel->parameters;

                        bool isDominated = false;
                        std::stack<std::shared_ptr<hierLabel>> toRemove;
                        for (auto &rLab : goalParetoSet) {
                            if (checkDomination(rLab->parameters, newParameters)) {
                                isDominated = true;
                                break;
                            } else if (checkDomination(newParameters, rLab->parameters)) {
                                toRemove.push(rLab);
                            }
                        }
                        if (isDominated)
                            continue;
                        while (!toRemove.empty()) {
                            auto pLab = toRemove.top();
                            toRemove.pop();
                            goalParetoSet.erase(pLab);
                        }
                        goalParetoSet.insert(currentLabel);
                    }
                }
            }
        }
        // Expand to neighbor vertices
        for (int edge : mCoverGraphs[currentLevel]->getConnectedOutgoingEdges(incomingEdge)) {
            const CoverEdge &rNextEdge = mCoverGraphs[currentLevel]->getCoverEdge(edge);

            int nextNode = rNextEdge.getEndNode();

            // Check for twice visited simple edges
            std::unordered_set<int> visitedEdges;
            bool twiceVisited = false;
            auto tempLabel = pCurrentLabel;
            while (tempLabel->pParent != nullptr) {
                for (int edge : tempLabel->usedEdge->getInsideEdges()) {
                    if (visitedEdges.count(edge) > 0) {
                        twiceVisited = true;
                        break;
                    }
                    visitedEdges.insert(edge);
                }

                tempLabel = tempLabel->pParent;
            }

            if (twiceVisited) {
                continue;
            }

            // if (nextNode == currentNode || (pCurrentLabel->pParent && nextNode == pCurrentLabel->pParent->node)) {
            //     continue;
            // }

            if (!mOriginalGraph.getNode(nextNode).inCluster(mClusterID)) {
                continue;
            }

            bool congested = false;
            // Prepare planning parameters for the next edge
            for (int e : rNextEdge.getInsideEdges()) {
                int currentInterval = timeToInterval(mCurrentTime + pCurrentLabel->parameters[1]);
                double dens = 0;
                if (currentInterval < mEdgeDensities[e].size()) {
                    dens = mEdgeDensities[e][currentInterval];
                }
                // double dens = extractDynamicParameters(mOriginalGraph.getEdge(e),
                //                                      mCurrentTime + pCurrentLabel->parameters[1])[3];
                if (dens > mCongestionThreshold) {
                    congested = true;
                    break;
                }
            }
            if (congested) {
                continue;
            }

            std::shared_ptr<hierLabel> nextLabel = expandLabel(pCurrentLabel, std::make_shared<CoverEdge>(rNextEdge));
            std::vector<double> nextParameters = nextLabel->parameters;

            int nextIncomingEdge = rNextEdge.getEndEdge();

            if (tDiscard(tSets[nextNode][nextIncomingEdge], nextParameters))
                continue;

            bool isDominated = false;
            std::stack<std::shared_ptr<hierLabel>> toRemove;
            for (auto &rLab : (*openLabels)[nextNode][nextIncomingEdge]) {
                if (checkDomination(rLab->parameters, nextParameters)) {
                    isDominated = true;
                    break;
                } else if (checkDomination(nextParameters, rLab->parameters)) {
                    toRemove.push(rLab);
                }
            }
            if (isDominated)
                continue;
            while (!toRemove.empty()) {
                auto pLab = toRemove.top();
                toRemove.pop();
                (*openLabels)[nextNode][nextIncomingEdge].erase(pLab);
                toExpand.erase(pLab);
            }

            (*openLabels)[nextNode][nextIncomingEdge].insert(nextLabel);
            toExpand.insert(nextLabel);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    logMessage("Elapsed time for complete search: " + std::to_string(elapsed.count()) + "s");

    return goalParetoSet;
}

/*
 * 1) Predicts densities in the future interval.
 * 2) Detects congestions in the future interval.
 * 3) Selects vehicles to reroute.
 * 4) Computes new routes for the rerouted vehicles.
 * 5) Updates inner routes and interval counts.
 * 6) Returns new routes.
 * */
std::vector<routeData> Router::reactivelyReroute(int currentTime) {
    std::unordered_map<int, double> predictedDensities;
    for (auto &e : mCoverGraphs[0]->getCoverEdges()) {
        int innerID = *e.getInsideEdges().begin();
        predictedDensities[innerID] = mTrafficEstimator.predictDensity(
            mOriginalGraph.getEdge(innerID), mEdgeFlows[innerID][0], mEdgeDensities[innerID][0]);
    }

    std::unordered_set<int> congestedEdges = predictCongestion(predictedDensities);
    std::unordered_set<int> vehiclesToReroute = selectForRerouting(congestedEdges);
    std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> reroutingDemand;

    logMessage("congestedEdges: " + std::to_string(congestedEdges.size()));
    logMessage("vehiclesToReroute: " + std::to_string(vehiclesToReroute.size()));

    std::unordered_map<int, int> afflictedEdges;

    for (int v : vehiclesToReroute) {
        // Get start edge, end edge, and vehicle priorities
        int startingPosition = mVehiclePositions.at(v);
        std::vector<int> remainingRoute(mVehicleRoutes[v].begin() + startingPosition, mVehicleRoutes[v].end());

        // Remove the rerouted vehicles from current flow registry
        for (int i = startingPosition + 1; i < mVehicleRoutes[v].size(); ++i) {
            int enterInterval = timeToInterval(mVehicleTimes[v][i - 1]);
            mEdgeFlows[mVehicleRoutes[v][i]][enterInterval].first -= 1;
            int exitInterval = timeToInterval(mVehicleTimes[v][i]);
            mEdgeFlows[mVehicleRoutes[v][i]][exitInterval].second -= 1;

            if (afflictedEdges.count(mVehicleRoutes[v][i]) == 0) {
                afflictedEdges[mVehicleRoutes[v][i]] = enterInterval;
            } else {
                if (enterInterval < afflictedEdges[mVehicleRoutes[v][i]]) {
                    afflictedEdges[mVehicleRoutes[v][i]] = enterInterval;
                }
            }
        }

        std::vector<double> priorities = mVehiclePriorities.at(v);
        reroutingDemand.emplace_back(v, remainingRoute, priorities);
    }

    // Update dynamic parameters on afflicted edges
    for (auto &[edge, interval] : afflictedEdges) {
        int currentInterval = interval;
        while (currentInterval <= mEdgeDensities[edge].size()) {
            // TODO: make sure update is conducted in correct interval
            auto [density, speed, fuel] =
                predictDynamicParameters(mOriginalGraph.getEdge(edge), currentInterval * mIntervalLength);

            if (fabs(mEdgeDensities[edge][currentInterval] - density) <= 0.01) {
                break;
            }

            mEdgeDensities[edge][currentInterval] = density;
            mEdgeSpeeds[edge][currentInterval] = speed;
            mEdgeFuels[edge][currentInterval] = fuel;
            currentInterval += 1;
        }
    }

    std::vector<routeData> newRoutes = proactivelyRoute(reroutingDemand, currentTime);

    // std::vector<routeData> newRoutes;

    nextInterval();

    return newRoutes;
}

/**
 *
 * @param currentPositions Current positions of vehicles in the cluster received from the simulator
 * @param currentDensities Current densities on all edges in the cluster received from the simulator
 * @param currentTime Current time of the simulation
 *
 * Main synchronization function that updates the positions of vehicles in the cluster and densities on all edges.
 * Computes expected positions of all vehicles and compares them to the real ones.
 * If an offset in position is detected, the function updates the time estimations of the route and edge flows.
 * The offsets are communicated only to the next cluster and forwarded further.
 * Afterward received offsets from other clusters and updates own estimations accordingly.
 * Dynamic parameters are updated according to the provided densities.
 */
void Router::synchronizePositions(const std::vector<positionData> &currentPositions,
                                  const std::unordered_map<int, double> &currentDensities, int currentTime) {
    /*
     * For every vehicle in currentPositions
     *  Analyze it's current position and compare it to predicted one
     *  Update current position and expectations for future positions
     *  Notify other ACUs on update offset
     */
    mCurrentTime = currentTime;
    int myID = mClusterID;
    auto findConnectedClusters = [myID](int mask) {
        std::unordered_set<int> connectedObjects;
        int index = 0;

        // Iterate through each bit in the bitmask
        while (mask != 0) {
            if ((mask & 1) && index != myID) {  // Check if bit is set and not the excludeID
                connectedObjects.insert(index);
            }
            mask >>= 1;
            index += 1;
        }

        return connectedObjects;
    };

    std::unordered_map<int, bool> updated;
    for (auto &p : mVehicleRoutes) {
        updated[p.first] = false;
    }
    std::unordered_map<int, std::unordered_map<int, double>> communicatedOffsets;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        if (i != mClusterID) {
            communicatedOffsets[i] = std::unordered_map<int, double>();
        }
    }

    mCurrentEdgePopulation.clear();

    // Fill edge populations
    // Find vehicle predicted/real positions and perform offsetting if necessary
    for (auto &pos : currentPositions) {
        int vehID = pos.vehicleID;

        if (mVehiclePositions.count(vehID) == 0) {
            continue;
        }

        int currentEdge = pos.edgeID;
        auto &currentRoute = mVehicleRoutes[vehID];
        auto &currentTimes = mVehicleTimes[vehID];

        int currentIndex =
            static_cast<int>(std::find(currentRoute.begin(), currentRoute.end(), currentEdge) - currentRoute.begin());

        // Find estimated index by the closest time
        int estimatedIndex = 0;
        double minDiff = std::numeric_limits<double>::max();
        for (int i = 0; i < currentTimes.size(); ++i) {
            double diff = std::abs(currentTimes[i] - (currentTime - simulationStart));
            if (diff < minDiff) {
                minDiff = diff;
                estimatedIndex = i;
            }
            if (minDiff < 1) {
                break;
            }
        }

        mVehiclePositions[vehID] = currentIndex;
        mCurrentEdgePopulation[currentEdge].insert(vehID);
        updated[vehID] = true;

        // We know current time, vehicles current position
        // We also know expected position
        // The difference between current time and expected time is the offset
        // Time expectations from the earliest (current or expected) edge are to be updated by the offset
        double expectedTime = currentTimes[currentIndex];
        double timeOffset = currentTime - (simulationStart + expectedTime);

        if (currentIndex != estimatedIndex) {
            // The offset is found, update the route estimations by the offset
            // ??? Compute traversal by time or just move according to offset ???
            std::set<int> involvedClusterBitmasks;
            int updateIndex = std::min(currentIndex, estimatedIndex);
            while (updateIndex < currentTimes.size()) {

                // Check if the edge is in own cluster for updating
                int currentCluster = mOriginalGraph.getEdge(currentRoute[updateIndex]).getClusterBitmask();
                if (currentCluster & (1 << mClusterID)) {
                    int expectedExit = currentTimes[updateIndex];
                    if (timeToInterval(expectedExit) >= 0) {
                        mEdgeFlows[currentRoute[updateIndex]][timeToInterval(expectedExit)].second -= 1;
                    }
                    int realExit = expectedExit + timeOffset;
                    int realExitInterval = timeToInterval(realExit);
                    if (realExitInterval < 0) {
                        realExitInterval = 0;
                    }
                    while (realExitInterval + 1 >= mEdgeFlows[currentRoute[updateIndex]].size()) {
                        mEdgeFlows[currentRoute[updateIndex]].push_back(std::make_pair(0, 0));
                    }

                    mEdgeFlows[currentRoute[updateIndex]][realExitInterval].second += 1;

                    if (updateIndex > 0) {
                        int expectedEnter = currentTimes[updateIndex - 1];
                        if (timeToInterval(expectedEnter) >= 0) {
                            mEdgeFlows[currentRoute[updateIndex]][timeToInterval(expectedEnter)].first -= 1;
                        }
                        int realEnter = expectedEnter + timeOffset;
                        int realEnterInterval = timeToInterval(realEnter);
                        if (realEnterInterval >= 0) {
                            while (realEnterInterval + 1 >= mEdgeFlows[currentRoute[updateIndex]].size()) {
                                mEdgeFlows[currentRoute[updateIndex]].push_back(std::make_pair(0, 0));
                            }
                            mEdgeFlows[currentRoute[updateIndex]][realEnterInterval].first += 1;
                        }
                    }
                }

                currentTimes[updateIndex] += timeOffset;
                if (!(currentCluster & (1 << mClusterID))) {
                    // The route continues in a different area, send the respective information
                    involvedClusterBitmasks.insert(currentCluster);
                }
                updateIndex += 1;
            }
            // Find clusters that are involved in the route and send them the offset update
            for (int c : involvedClusterBitmasks) {
                auto cIDs = findConnectedClusters(c);
                for (int id : cIDs) {
                    communicatedOffsets[id][vehID] = timeOffset;
                }
            }
        }
    }
    // Send the vehicle and its offset to next ACUs
    for (int n = 0; n < NUM_AGENTS; ++n) {
        if (n == mClusterID)
            continue;
        // Check if there are any offsets to send
        std::unordered_map<int, double> offsets;
        if (communicatedOffsets.count(n) > 0) {
            offsets = communicatedOffsets[n];
        }
        std::vector<double> combinedData;
        combinedData.reserve(offsets.size() * 2 + 1);
        // First element is total number of key-value pairs
        combinedData.push_back(static_cast<double>(offsets.size()));
        // Then alternate key and value
        for (auto &[k, v] : offsets) {
            combinedData.push_back(static_cast<double>(k));
            combinedData.push_back(v);
        }
        // Single send
        MPI_Bsend(combinedData.data(), static_cast<int>(combinedData.size()), MPI_DOUBLE, n + 1,
                  static_cast<int>(MessageTags::addOffset), MPI_COMM_WORLD);
    }

    // Wait for messages from other ACUs to update vehicles incoming from them
    for (int n = 0; n < NUM_AGENTS; ++n) {
        if (n == mClusterID)
            continue;

        MPI_Status status;
        int numElements;
        MPI_Probe(n + 1, static_cast<int>(MessageTags::addOffset), MPI_COMM_WORLD, &status);
        MPI_Get_count(&status, MPI_INT, &numElements);

        std::vector<double> receivedData(numElements);
        MPI_Recv(receivedData.data(), numElements, MPI_DOUBLE, n + 1, static_cast<int>(MessageTags::addOffset),
                 MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    }

    // Go through recorded vehicle routes
    // If a vehicle was not updated, it is removed due to being out of system/cluster
    for (auto &[vehID, wasUpdated] : updated) {
        if (!wasUpdated && mVehiclePositions.count(vehID) > 0) {
            // Create iterator at the position of the vehicle
            auto lastPosition = mVehicleRoutes[vehID].begin() + mVehiclePositions[vehID];
            bool leftArea = false;
            bool willReturn = false;
            while (lastPosition < mVehicleRoutes[vehID].end()) {
                if (!leftArea && !mOriginalGraph.getEdge(*lastPosition).inCluster(mClusterID)) {
                    leftArea = true;
                    lastPosition += 1;
                    continue;
                }
                if (leftArea && mOriginalGraph.getEdge(*lastPosition).inCluster(mClusterID)) {
                    willReturn = true;
                    break;
                }
                lastPosition += 1;
            }
            if (lastPosition == mVehicleRoutes[vehID].end() || (leftArea && !willReturn)) {
                mVehicleRoutes.erase(vehID);
                mVehicleTimes.erase(vehID);
                mVehiclePositions.erase(vehID);
                mVehiclePriorities.erase(vehID);
            }
        }
    }

    // Record current densities
    for (auto &[edge, density] : currentDensities) {
        mEdgeDensities[edge][0] = density;
        mEdgeSpeeds[edge][0] = mSpeedEstimator.predictSpeed(mOriginalGraph.getEdge(edge), density);
        mEdgeFuels[edge][0] = mFuelEstimator.predictFuel(mOriginalGraph.getEdge(edge), density, mEdgeSpeeds[edge][0]);
    }
}

/**
 *
 * @param currentLabel Current label to be expanded
 * @param nextEdge Next edge to be added to the route
 * @return New label expanded via the provided edge
 *
 * Label expansion function that extends the route by adding the next edge to the path.
 * The dynamic extension parameters are taken from prepareDynamicParameters function.
 */
std::shared_ptr<hierLabel> Router::expandLabel(const std::shared_ptr<hierLabel> &currentLabel,
                                               const std::shared_ptr<CoverEdge> &nextEdge) {
    std::vector<double> parameters = currentLabel->parameters;
    int numHops = currentLabel->numHops;
    for (int e : nextEdge->getInsideEdges()) {
        std::vector<double> suffixParameters =
            extractDynamicParameters(mOriginalGraph.getEdge(e), mCurrentTime + parameters[1]);
        parameters = linkPaths(parameters, numHops, suffixParameters, 1);
        numHops += 1;
    }
    return std::make_shared<hierLabel>(nextEdge->getEndNode(), parameters, currentLabel, nextEdge, numHops);
}

/**
 * Function called when an interval comes to an end.
 * Increments own interval counters.
 * Deletes the first (ending) interval from own memory and adds a new empty one at the end.
 */
void Router::nextInterval() {
    // For every edge in the interval, remove the first interval and add a new one at the end
    mCurrentInterval += 1;
    mPassedIntervals += 1;
    // Update the edge flows
    for (auto &p : mEdgeFlows) {
        // Remove the first element of the vector
        p.second.erase(p.second.begin());
        // Add a new element at the end
        p.second.emplace_back(0, 0);
    }
    // Update the edge densities
    for (auto &p : mEdgeDensities) {
        // Remove the first element of the vector
        p.second.erase(p.second.begin());
        // Add a new element at the end
        p.second.push_back(0);
    }
    // Update the edge speeds
    for (auto &p : mEdgeSpeeds) {
        // Remove the first element of the vector
        p.second.erase(p.second.begin());
        // Add a max speed element at the end
        p.second.push_back(mOriginalGraph.getEdge(p.first).getSpeedLimit());
    }
    // Update the edge fuels
    std::vector<const Edge *> edgesToPredict;
    std::vector<double> densitiesToPredict;
    std::vector<double> speedsToPredict;
    std::unordered_map<int, int> mapIDs;

    for (auto &p : mEdgeFuels) {
        // Remove the first element of the vector
        p.second.erase(p.second.begin());
        // Prepare for batch prediction
        edgesToPredict.push_back(&mOriginalGraph.getEdge(p.first));
        densitiesToPredict.push_back(0);
        speedsToPredict.push_back(mEdgeSpeeds[p.first].back());
        mapIDs[p.first] = edgesToPredict.size() - 1;
    }

    // Perform batch prediction
    std::vector<double> fuels = mFuelEstimator.predictFuelBatch(edgesToPredict, densitiesToPredict, speedsToPredict);

    // Update fuel values
    for (auto &p : mEdgeFuels) {
        p.second.push_back(fuels[mapIDs[p.first]]);
    }
}

void Router::logMessage(const std::string &message) const {
    if (mEnableLogging) {
        std::cout << "LOG|" << mClusterID << "|" << message << std::endl;
    }
}

void Router::logError(const std::string &error) const {
    if (mEnableLogging) {
        std::cerr << "ERR|" << mClusterID << "|" << error << std::endl;
    }
}
