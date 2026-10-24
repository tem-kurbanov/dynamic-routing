// Dependency-free fixture for production selection and reactive orchestration.
// The runner extracts the methods from Router.cpp; Graph and Edge are real types.
#include <chrono>
#include <cmath>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include "Graph.h"
#include "serializations.h"

struct TrafficEstimator {
    double predictDensity(const Edge &edge, std::pair<int, int>, double) {
        return edge.getID() == 0 ? 1 : 0;
    }
};

struct CoverGraph {
    std::vector<int> getInsideEdges() const { return {0}; }
    std::vector<CoverGraph> getCoverEdges() const { return {*this}; }
};

struct Router {
    Graph &mOriginalGraph;
    int mClusterID = 0;
    double mIntervalLength = 60;
    double mReroutePortion = 1;
    int mPassedIntervals = 0;
    TrafficEstimator mTrafficEstimator;
    CoverGraph cover;
    std::vector<CoverGraph *> mCoverGraphs{&cover};
    std::unordered_map<int, std::vector<std::pair<int, int>>> mEdgeFlows;
    std::unordered_map<int, std::vector<double>> mEdgeDensities, mEdgeFuels, mVehiclePriorities;
    std::unordered_map<int, std::vector<double>> mEdgeSpeeds, mVehicleTimes;
    std::unordered_map<int, std::vector<int>> mVehicleRoutes;
    std::unordered_map<int, int> mVehiclePositions;
    std::unordered_map<int, std::vector<std::pair<bool, bool>>> mVehicleFlowRecorded;
    std::unordered_map<int, std::unordered_set<int>> mCurrentEdgePopulation;
    std::unordered_set<int> selectForRerouting(const std::unordered_set<int> &);
    std::vector<routeData> reactivelyReroute(int);
    int timeToInterval(double);
    int timeToInterval(int);
    std::vector<int> forecastIntervals;
    bool advanced = false;
    std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> demand;
    std::unordered_set<int> predictCongestion(const std::unordered_map<int, double> &) { return {0}; }
    std::tuple<double, double, double> predictDynamicParameters(const Edge &, int time) {
        int interval = timeToInterval(time);
        if (interval < 1 || interval > 2) throw std::runtime_error("Incorrect forecast timestamp");
        forecastIntervals.push_back(interval);
        return {0.5, 5, 2};
    }
    std::vector<routeData> proactivelyRoute(
        const std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> &requests, int) {
        demand = requests;
        return {routeData(42, std::get<1>(requests.at(0)))};
    }
    void nextInterval() { advanced = true; }
    void logMessage(const std::string &) {}
};

#include "reroute_selection.inc"

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    const std::string test = argv[1];
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    // Edge 0 is congested; edge 1 feeds it; edge 2 feeds edge 1.
    for (int id = 0; id < 3; ++id) {
        std::unordered_set<int> incoming, outgoing;
        if (id < 2) incoming.insert(id + 1);
        edges.emplace_back(id, 0, 0, 0, 10, 10, 1, 1, 0, incoming, outgoing, 1);
    }
    Graph graph(nodes, edges);
    Router router{graph};
    for (int id = 0; id < 3; ++id) router.mEdgeSpeeds[id] = {10};
    router.mCurrentEdgePopulation[1] = {42};
    router.mVehicleRoutes[42] = {1, 0};
    router.mVehicleTimes[42] = {0, 1};
    router.mVehiclePositions[42] = 0;
    std::unordered_set<int> congestion{0}, expected{42};

    if (test.rfind("reactive_", 0) == 0) {
        // Exercise real reactive orchestration; prediction/replanning are boundaries.
        router.mPassedIntervals = 2;
        router.mVehiclePriorities[42] = {1, 2, 3, 4};
        router.mVehicleTimes[42] = {180, 190};
        if (test == "reactive_expired_inflow") router.mVehicleTimes[42] = {120, 130};
        if (test == "reactive_foreign") {
            std::vector<Node> otherNodes;
            std::vector<Edge> otherEdges;
            for (int id = 0; id < 3; ++id) {
                std::unordered_set<int> in, out;
                if (id == 0) in.insert(1);
                otherEdges.emplace_back(id, 0, 0, 0, 10, 10, 1, 1, 0, in, out, id == 2 ? 2 : 1);
            }
            graph = Graph(otherNodes, otherEdges);
            router.mVehicleRoutes[42] = {1, 0, 2};
            router.mVehicleTimes[42] = {180, 190, 200};
            router.mEdgeSpeeds.erase(2);
        }
        for (int id : {0, 1}) {
            router.mEdgeFlows[id].assign(3, {1, 1});
            router.mEdgeDensities[id].assign(3, 0);
            router.mEdgeSpeeds[id].assign(3, 10);
            router.mEdgeFuels[id].assign(3, 1);
        }
        router.mVehicleFlowRecorded[42].assign(router.mVehicleRoutes[42].size(), {true, true});
        const auto routes = router.reactivelyReroute(180);
        if (routes.size() != 1 || routes[0].vehicleID != 42 || !router.advanced ||
            router.demand.size() != 1 || std::get<0>(router.demand[0]) != 42 ||
            router.forecastIntervals != std::vector<int>{1, 2} || router.mEdgeFlows.count(2) != 0)
            throw std::runtime_error("Reactive routing did not hand off the selected vehicle correctly");
        const bool expired = test == "reactive_expired_inflow";
        if (router.mEdgeFlows[0][0].first != (expired ? 1 : 0) ||
            router.mEdgeFlows[0][expired ? 0 : 1].second != 0)
            throw std::runtime_error("Incorrect removal of recorded flows");
        return 0;
    } else if (test == "eligible") {
        // Default fixture: select the upstream vehicle at a 100% portion.
    } else if (test == "no_congestion") {
        congestion.clear(); expected.clear();
    } else if (test == "zero_portion") {
        router.mReroutePortion = 0; expected.clear();
    } else if (test == "unaffected_route") {
        router.mVehicleRoutes[42] = {1, 2}; expected.clear();
    } else if (test == "outside_horizon") {
        router.mVehicleTimes[42] = {0, 60}; expected.clear();
    } else if (test == "upstream_congested") {
        congestion.insert(1); expected.clear();
    } else if (test == "foreign_edge") {
        std::vector<Node> otherNodes;
        std::vector<Edge> otherEdges;
        for (int id = 0; id < 3; ++id) {
            std::unordered_set<int> in, out;
            if (id < 2) in.insert(id + 1);
            otherEdges.emplace_back(id, 0, 0, 0, 10, 10, 1, 1, 0, in, out, id == 1 ? 2 : 1);
        }
        graph = Graph(otherNodes, otherEdges);
        router.mEdgeSpeeds.erase(1); expected.clear();
    } else if (test == "zero_length_cycle") {
        std::vector<Node> cycleNodes;
        std::vector<Edge> cycleEdges;
        for (int id = 0; id < 3; ++id) {
            std::unordered_set<int> in{id == 0 ? 1 : (id == 1 ? 2 : 1)}, out;
            cycleEdges.emplace_back(id, 0, 0, 0, 0, 10, 1, 1, 0, in, out, 1);
        }
        graph = Graph(cycleNodes, cycleEdges);
    } else {
        throw std::runtime_error("Unknown test: " + test);
    }
    if (router.selectForRerouting(congestion) != expected)
        throw std::runtime_error(test + ": unexpected selected vehicle set");
}
