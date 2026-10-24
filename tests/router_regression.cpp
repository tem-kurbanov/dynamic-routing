// Exercise the production Router with real graphs/models and real MPI messages.
#include "Router.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
template<class F> void rejects(F function, const char *message) {
    try { function(); } catch (const std::exception &) { return; }
    throw std::runtime_error(message);
}

Graph makeGraph() {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    for (int n = 0; n < 5; ++n) {
        std::unordered_set<int> in, out;
        if (n > 0) in.insert(n - 1);
        if (n < 4) out.insert(n);
        nodes.emplace_back(n, std::pair<double, double>{n, 0}, in, out, 3);
    }
    for (int e = 0; e < 4; ++e) {
        std::unordered_set<int> in, out;
        if (e > 0) in.insert(e - 1);
        if (e < 3) out.insert(e + 1);
        edges.emplace_back(e, e, e + 1, 1, 1.5, 1, 1, 1, 0, in, out, 3);
    }
    return Graph(nodes, edges);
}

struct RouterTestAccess {
    static void state(Router &r) {
        r.setLogging(false);
        r.mPassedIntervals = 2;
        require(r.timeToInterval(120) == -1, "Expired interval must be negative");
        require(r.timeToInterval(121) == 0 && r.timeToInterval(180) == 0 &&
                r.timeToInterval(181.0) == 1, "Interval boundaries changed");
        require(r.timeToInterval(120.5) == -1, "Fractional expired interval");
        r.mVehicleRoutes[42] = {0, 1, 2};
        r.mVehicleTimes[42] = {150.5, 190.5, 240.5};
        r.mVehicleFlowRecorded[42].assign(3, {true, true});
        for (int e = 0; e < 4; ++e) r.mEdgeFlows[e].assign(5, {0, 0});
        r.mEdgeFlows[0][0].second = 1;
        r.mEdgeFlows[1][0].first = 1;
        r.mEdgeFlows[1][1].second = 1;
        r.mEdgeFlows[2][1].first = 1;
        r.mEdgeFlows[2][1].second = 1;
        r.applyTimeOffset(42, 60.0, 0);
        require(r.mVehicleTimes[42] == std::vector<double>({210.5, 250.5, 300.5}), "Timestamp shifted twice or truncated");
        require(r.mEdgeFlows[1][0].first == 0 && r.mEdgeFlows[1][1].first == 1 &&
                r.mEdgeFlows[1][2].first == 0, "Inflow must move from immutable predecessor time");
        require(r.mEdgeFlows[2][1].first == 0 && r.mEdgeFlows[2][2].first == 1, "Next inflow shifted incorrectly");
        require(r.mEdgeFlows[0][0].second == 0 && r.mEdgeFlows[0][1].second == 1, "Outflow not moved");
        r.mVehicleRoutes[43] = {3};
        r.mVehicleTimes[43] = {100.5};
        r.mVehicleFlowRecorded[43].assign(1, {false, true});
        r.applyTimeOffset(43, 100, 0);
        require(std::all_of(r.mEdgeFlows[3].begin(), r.mEdgeFlows[3].end(), [](auto flow) {
            return flow == std::pair{0, 0};
        }), "Expired event resurrected");
        r.mEdgeFlows[3][1].second = 7; // Other vehicles own these aggregate events.
        r.applyTimeOffset(43, 60, 0);
        require(r.mEdgeFlows[3][1].second == 7 && r.mEdgeFlows[3][2].second == 0,
                "Repeated offset recreated an expired event or stole another vehicle's flow");
        r.mEdgeFlows[3][1].second = 0;
        r.mVehicleFlowRecorded[43].assign(1, {false, true});
        r.mVehicleTimes[43] = {150.5};
        r.mEdgeFlows[3][0].second = 1;
        r.applyTimeOffset(43, -60, 0);
        require(r.mEdgeFlows[3][0].second == 0, "Event pulled into the past must leave forecast");
        r.applyTimeOffset(43, 120, 0);
        require(std::all_of(r.mEdgeFlows[3].begin(), r.mEdgeFlows[3].end(), [](auto flow) {
            return flow == std::pair{0, 0};
        }), "Pulled-past event was recreated by a later delay");
        r.applyTimeOffset(999, 1, 0); // Unknown vehicle is harmless.

        auto root = std::make_shared<hierLabel>(1, std::vector<double>(4), nullptr,
                                                std::make_shared<CoverEdge>(r.mOriginalGraph.getEdge(0)), 1);
        auto child = std::make_shared<hierLabel>(2, std::vector<double>(4), root,
                                                 std::make_shared<CoverEdge>(r.mOriginalGraph.getEdge(1)), 2);
        require(r.reusesOriginalEdge(child, CoverEdge(r.mOriginalGraph.getEdge(0))), "Root-edge cycle missed");
        require(r.reusesOriginalEdge(child, CoverEdge(r.mOriginalGraph.getEdge(1))), "Immediate edge reuse missed");
        require(!r.reusesOriginalEdge(child, CoverEdge(r.mOriginalGraph.getEdge(2))), "Valid extension rejected");
        require(r.reusesOriginalEdge(child, CoverEdge(9, 0, {2, 2})), "Internal cover-edge cycle missed");

        // Both agents can find a shared destination. Ownership belongs to each candidate.
        RouteCandidate good{{1, 1, 1, 1}, {0, 2, 1}};
        RouteCandidate poor{{10, 10, 10, 10}, {0, 1, 2}};
        auto selected = r.selectBestCandidate({good, poor}, {1, 2, 3, 4});
        require(selected.clusterSequence == good.clusterSequence, "Selected route lost its finder/search context");
        auto tie = r.selectBestCandidate({good, {good.parameters, poor.clusterSequence}}, {1, 2, 3, 4});
        require(tie.clusterSequence == good.clusterSequence, "Equal criteria changed candidate provenance");

        r.mPassedIntervals = 0;
        for (int e = 0; e < 4; ++e) {
            r.mEdgeDensities[e].assign(60, 0);
            r.mEdgeSpeeds[e].assign(60, 1);
            r.mEdgeFuels[e].assign(60, 1);
            r.mEdgeFlows[e].assign(60, {0, 0});
        }
        r.recordRoute(44, {0, 1, 2}, {}, {1, 2, 3, 4}, 1.0);
        require(r.mVehicleTimes[44] == std::vector<double>({2.5, 4.0, 5.5}), "Fractional route times truncated");
        r.recordRoute(45, {}, {}, {}, 1); // Empty demand cannot index the route.
        r.mVehiclePositions[44] = 0;
        r.synchronizePositions({positionData(44, 3)}, {}, 20);
        require(r.mVehicleRoutes.count(44) == 1 && r.mVehiclePositions[44] == 0,
                "Stale observed edge deleted vehicle or stored an invalid index");
        r.mVehicleTimes[44].clear();
        r.mCurrentEdgePopulation[1] = {44};
        require(r.selectForRerouting({2}).empty(), "Inconsistent route state selected");
        std::cout << "PASS router state: intervals, flow shifts, cycles, provenance, fractions, stale state\n";
    }

    static void search(TrafficEstimator &density, SpeedEstimator &speed, FuelEstimator &fuel) {
        Graph graph = makeGraph();
        Router valid(0, 1, 60, graph, 1, density, speed, fuel);
        valid.setLogging(false);
        valid.mCurrentTime = 1;
        auto paths = valid.completeHierarchicalSearch(0, 3);
        require(!paths.empty(), "Valid simple route rejected by loop prevention");
        std::vector<int> path;
        auto root = valid.buildForwardPath(*paths.begin(), path);
        path.insert(path.begin(), root->usedEdge->getInsideEdges().begin(), root->usedEdge->getInsideEdges().end());
        require(path == std::vector<int>({0, 1, 2, 3}), "Simple search returned an incorrect path");
        require((*paths.begin())->parameters[3] == 4, "Edge-count objective changed");

        std::vector<Node> nodes;
        std::vector<Edge> edges;
        for (int n = 0; n < 2; ++n) {
            std::unordered_set<int> in{1 - n}, out{n};
            nodes.emplace_back(n, std::pair<double, double>{n, 0}, in, out, 1);
            std::unordered_set<int> incoming{1 - n}, outgoing{1 - n};
            edges.emplace_back(n, n, 1 - n, 1, 10, 10, 1, 1, 0, incoming, outgoing, 1);
        }
        Graph cycle(nodes, edges);
        Router cyclic(0, 1, 60, cycle, 1, density, speed, fuel);
        cyclic.setLogging(false);
        cyclic.mCurrentTime = 1;
        auto first = std::make_shared<hierLabel>(1, std::vector<double>{10, 1, 1, 1}, nullptr,
                                                std::make_shared<CoverEdge>(cycle.getEdge(0)), 1);
        auto last = std::make_shared<hierLabel>(0, std::vector<double>{20, 2, 2, 2}, first,
                                               std::make_shared<CoverEdge>(cycle.getEdge(1)), 2);
        std::unordered_map<int, std::unordered_map<int, std::set<std::shared_ptr<hierLabel>, hierLexCompare>>> open;
        open[0][1].insert(last);
        require(cyclic.completeHierarchicalSearch(-1, 0, open).empty(),
                "Goal connection repeated an original edge");
        std::cout << "PASS actual search: valid path, unchanged edge-count objective, goal-cycle rejection\n";
    }

    static void wire(Router &r, int rank) {
        const std::vector<RouteCandidate> candidates{
            {{1, 2, 3, 4}, {0, 1}}, {{2, 3, 4, 5}, {0, 2, 1}}, {{3, 4, 5, 6}, {0, 1}}};
        if (rank == 1) r.sendFoundRoutes(1, 42, {0}, candidates);
        if (rank == 2) {
            MPI_Status status;
            MPI_Probe(1, static_cast<int>(MessageTags::foundRoutes), MPI_COMM_WORLD, &status);
            auto [vehicle, sequence, routes] = r.receiveFoundRoutes(0, status);
            require(vehicle == 42 && sequence == std::vector<int>{0} && routes.size() == 3,
                    "Lost candidates in MPI foundRoutes");
            for (int i = 0; i < 3; ++i) require(routes[i].parameters == candidates[i].parameters &&
                routes[i].clusterSequence == candidates[i].clusterSequence, "MPI candidate provenance mismatch");
            r.sendAcceptedRoute(0, 42, {1, 2, 3, 4}, routes[1]);
        }
        if (rank == 1) {
            MPI_Status status;
            MPI_Probe(2, static_cast<int>(MessageTags::acceptedRoute), MPI_COMM_WORLD, &status);
            auto [vehicle, priorities, route] = r.receiveAcceptedRoute(1, status);
            require(vehicle == 42 && route.clusterSequence == candidates[1].clusterSequence &&
                    route.parameters == candidates[1].parameters, "Accepted route lost search sequence");
        }
        MPI_Barrier(MPI_COMM_WORLD);
        if (rank == 1) {
            std::vector<double> truncated{42, 1, 0, 3, 1, 2, 3};
            MPI_Bsend(truncated.data(), truncated.size(), MPI_DOUBLE, 2,
                      static_cast<int>(MessageTags::foundRoutes), MPI_COMM_WORLD);
        }
        if (rank == 2) {
            MPI_Status status;
            MPI_Probe(1, static_cast<int>(MessageTags::foundRoutes), MPI_COMM_WORLD, &status);
            rejects([&] { r.receiveFoundRoutes(0, status); }, "Truncated candidate message accepted");
        }
        MPI_Barrier(MPI_COMM_WORLD);
        // Real synchronizePositions communication: rank 1 sends a delay to rank 2.
        if (rank > 0) {
            r.mPassedIntervals = 0;
            r.mVehicleRoutes[42] = {0, 1, 2};
            r.mVehicleTimes[42] = {30, 70, 110};
            r.mVehicleFlowRecorded[42].assign(3, {true, true});
            r.mVehiclePositions[42] = 0;
            r.mEdgeFlows[0][0].second = 1;
            r.mEdgeFlows[1][0].first = 1;
            r.mEdgeFlows[1][1].second = 1;
            r.mEdgeFlows[2][1].first = 1;
            r.mEdgeFlows[2][1].second = 1;
            r.synchronizePositions(rank == 1 ? std::vector<positionData>{{42, 0}} : std::vector<positionData>{}, {}, 80);
            require(r.mVehicleTimes[42] == std::vector<double>({80, 120, 160}), "Remote offset discarded or applied twice");
            require(r.mEdgeFlows[1][0].first == 0 && r.mEdgeFlows[1][1].first == 1,
                    "Remote inflow not shifted");
        }
        MPI_Barrier(MPI_COMM_WORLD);
        if (rank > 0) {
            // Both peers observe the same shared border vehicle: use the local
            // observation once, rather than also applying the peer's correction.
            r.synchronizePositions({positionData(42, 0)}, {}, 130);
            require(r.mVehicleTimes[42] == std::vector<double>({130, 170, 210}),
                    "Shared-border observations applied an offset twice");
        }
        MPI_Barrier(MPI_COMM_WORLD);
        if (rank == 0) std::cout << "PASS real MPI: three candidates, acceptance provenance, remote timing offset\n";
    }
};

int main(int argc, char **argv) {
    MPI_Init(&argc, &argv);
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    try {
        require(argc >= 2, "Expected scenario directory");
        const auto models = std::filesystem::path(argv[1]) / "xgrid/models/log";
        TrafficEstimator density((models / "density_60.json").string());
        SpeedEstimator speed((models / "speed_60.json").string());
        FuelEstimator fuel((models / "fuel_60.json").string());
        Graph graph = makeGraph();
        const bool mpi = argc > 2;
        Router router(mpi && rank > 0 ? rank - 1 : 0, mpi ? 2 : 1, 60, graph, 1, density, speed, fuel);
        router.setLogging(false);
        if (mpi) {
            std::vector<char> buffer(1024 * 1024);
            MPI_Buffer_attach(buffer.data(), buffer.size());
            RouterTestAccess::wire(router, rank);
            void *detached;
            int size;
            MPI_Buffer_detach(&detached, &size);
        } else {
            RouterTestAccess::state(router);
            RouterTestAccess::search(density, speed, fuel);
            const Edge &edge = graph.getEdge(0);
            rejects([&] { SpeedEstimator invalid("missing-model.json"); }, "Missing model accepted");
            rejects([&] { SpeedEstimator invalid((models / "fuel_60.json").string()); }, "Wrong model shape accepted");
            rejects([&] { speed.predictSpeed(edge, std::numeric_limits<double>::quiet_NaN()); }, "NaN feature accepted");
            std::vector<const Edge *> edges{&edge};
            std::vector<double> empty, densities{0.2}, speeds{1};
            rejects([&] { fuel.predictFuelBatch(edges, empty, speeds); }, "Mismatched batch accepted");
            edges[0] = nullptr;
            rejects([&] { fuel.predictFuelBatch(edges, densities, speeds); }, "Null batch edge accepted");
            edges.clear();
            require(fuel.predictFuelBatch(edges, empty, empty).empty(), "Empty batch failed");
            // Explicit original feature order and inverse transforms protect the supplied-model contract.
            auto rawDensity = estimator::loadModel((models / "density_60.json").string(), 8);
            auto rawSpeed = estimator::loadModel((models / "speed_60.json").string(), 6);
            auto rawFuel = estimator::loadModel((models / "fuel_60.json").string(), 5);
            const auto d = estimator::predict(rawDensity.get(), {2, 1, 1, 1.5F, 1, 1, 1, 0.2F}, 1, 8)[0];
            const auto s = estimator::predict(rawSpeed.get(), {1, 1.5F, 1, 1, 1, 0.2F}, 1, 6)[0];
            const auto f = estimator::predict(rawFuel.get(), {0.2F, 1, 1, 1.5F, 1}, 1, 5)[0];
            require(std::abs(density.predictDensity(edge, {2, 1}, 0.2) - 1 / (1 + std::exp(-d))) < 1e-12,
                    "Density feature order or sigmoid transform changed");
            require(std::abs(speed.predictSpeed(edge, 0.2) - std::max(0.01, std::expm1(s))) < 1e-12,
                    "Speed feature order or inverse-log transform changed");
            const auto expectedFuel = std::max(0.01, std::expm1(f));
            require(std::abs(fuel.predictFuel(edge, 0.2, 1) - expectedFuel) < 1e-12,
                    "Fuel feature order or inverse-log transform changed");
            edges = {&edge};
            require(std::abs(fuel.predictFuelBatch(edges, densities, speeds).at(0) - expectedFuel) < 1e-12,
                    "Fuel batch and scalar predictions differ");
            rejects([] { estimator::inverseLog(1000); }, "Overflowed inverse transform accepted");
            std::cout << "PASS estimator contracts: original features/transforms and scalar/batch equivalence\n";
            std::cout << "PASS estimator failures: missing/wrong models, nonfinite features, invalid/empty batches\n";
        }
    } catch (const std::exception &error) {
        std::cerr << "FAIL rank " << rank << ": " << error.what() << '\n';
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }
    MPI_Finalize();
}
