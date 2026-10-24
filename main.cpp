/**
 * @file main.cpp
 * @brief Entry point: orchestrates SUMO simulation and distributed routing via MPI.
 *
 * Master process controls SUMO and synchronization; ACU processes handle routing
 * using hierarchical MLS with dynamic estimators.
 */
#include <libsumo/libtraci.h>
#include <mpi.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "CoverGraph.h"
#include "Edge.h"
#include "FuelEstimator.h"
#include "Graph.h"
#include "Node.h"
#include "Router.h"
#include "SpeedEstimator.h"
#include "serializations.h"

int NUM_AGENTS = 4;
int NUM_PARAMETERS = 4;
bool ENABLE_LOGGING = false;

void logMessage(const std::string &message) {
    if (ENABLE_LOGGING) {
        std::cout << "LOG|MASTER|" << message << std::endl;
    }
}

void logError(const std::string &error) {
    std::cerr << "ERR|MASTER|" << error << std::endl;
}

std::tuple<std::vector<Node>, std::vector<Edge>> readSumoNetwork(const std::string &nodePath,
                                                                 const std::string &edgePath) {
    std::vector<Node> originalNodes;
    std::vector<Edge> originalEdges;

    // Read nodes from the nodePath
    std::ifstream nodeFile(nodePath);
    if (!nodeFile) {
        std::cerr << "Unable to open file " << nodePath << std::endl;
        exit(1);
    }
    std::string line;
    std::getline(nodeFile, line);
    while (std::getline(nodeFile, line)) {
        int id;
        double x, y;
        std::unordered_set<int> incomingEdges;
        std::unordered_set<int> outgoingEdges;
        // All attributes are separated by semicolons
        // Example: 0;(2539.61, 1935.66);[108, 224, 259];[25, 258, 498];0
        std::stringstream ss(line);
        std::string attribute;
        std::getline(ss, attribute, ';');
        id = stoi(attribute);
        std::getline(ss, attribute, ';');
        // Remove the parentheses
        attribute = attribute.substr(1, attribute.size() - 2);
        // Split the string by comma
        std::stringstream ss2(attribute);
        std::string xStr, yStr;
        std::getline(ss2, xStr, ',');
        std::getline(ss2, yStr, ',');
        x = stof(xStr);
        y = stof(yStr);
        std::getline(ss, attribute, ';');
        attribute = attribute.substr(1, attribute.size() - 2);
        std::stringstream ss3(attribute);
        while (std::getline(ss3, attribute, ',')) {
            incomingEdges.insert(stoi(attribute));
        }
        std::getline(ss, attribute, ';');
        attribute = attribute.substr(1, attribute.size() - 2);
        std::stringstream ss4(attribute);
        while (std::getline(ss4, attribute, ',')) {
            outgoingEdges.insert(stoi(attribute));
        }
        std::getline(ss, attribute, '\r');
        int cluster = std::stoi(attribute);

        originalNodes.emplace_back(id, std::make_pair(x, y), incomingEdges, outgoingEdges, cluster);
    }
    nodeFile.close();

    // Read edges from the edgePath
    std::ifstream edgeFile(edgePath);
    if (!edgeFile) {
        std::cerr << "Unable to open file " << edgePath << std::endl;
        exit(1);
    }
    std::getline(edgeFile, line);
    while (std::getline(edgeFile, line)) {
        // Example:
        // ID;FromNode;ToNode;Type;Length;Speed;Lanes;Incoming;Outgoing;ClusterID
        //          0;39;37;1;218.66;8.33;1;[11, 427, 43];[236, 368, 427];0
        int id;
        int startNode, endNode;
        int type;
        double length;
        double maxSpeed;
        int numLanes;
        double tlsCoeff;
        double dangerFactor;
        std::unordered_set<int> incoming;
        std::unordered_set<int> outgoing;
        std::stringstream ss(line);
        std::string attribute;
        std::getline(ss, attribute, ';');
        id = stoi(attribute);
        std::getline(ss, attribute, ';');
        startNode = stoi(attribute);
        std::getline(ss, attribute, ';');
        endNode = stoi(attribute);
        std::getline(ss, attribute, ';');
        type = stoi(attribute);
        std::getline(ss, attribute, ';');
        length = stof(attribute);
        std::getline(ss, attribute, ';');
        maxSpeed = stof(attribute);
        std::getline(ss, attribute, ';');
        numLanes = stoi(attribute);
        std::getline(ss, attribute, ';');
        tlsCoeff = stof(attribute);
        std::getline(ss, attribute, ';');
        dangerFactor = stof(attribute);
        std::getline(ss, attribute, ';');
        attribute = attribute.substr(1, attribute.size() - 2);
        std::stringstream ss2(attribute);
        while (std::getline(ss2, attribute, ',')) {
            incoming.insert(stoi(attribute));
        }
        std::getline(ss, attribute, ';');
        attribute = attribute.substr(1, attribute.size() - 2);
        std::stringstream ss3(attribute);
        while (std::getline(ss3, attribute, ',')) {
            outgoing.insert(stoi(attribute));
        }
        std::getline(ss, attribute, '\r');
        int cluster = std::stoi(attribute);
        originalEdges.emplace_back(id, startNode, endNode, type, length, maxSpeed, numLanes, tlsCoeff, dangerFactor,
                                   incoming, outgoing, cluster);
    }
    edgeFile.close();

    return {originalNodes, originalEdges};
}

std::vector<CoverGraph> buildCovers(const Graph &originalGraph, int k, int cluster) {
    std::vector<CoverGraph> covers;
    covers.emplace_back(originalGraph);
    int initialNumVertices = originalGraph.getNumNodes();
    for (int i = 1; i <= k; ++i) {
        std::cout << "Cover " << i << std::endl;
        covers.emplace_back(covers[i - 1]);
        double remainingVertexPercentage =
            static_cast<double>(covers[i].getNumNodes()) / static_cast<double>(initialNumVertices);
        if (remainingVertexPercentage < 0.01) {
            break;
        }
    }
    return covers;
}

std::vector<double> generatePriorities(const std::string &vehID) {
    // Read vehicle type from SUMO
    std::string vehType = libtraci::Vehicle::getTypeID(vehID);
    std::vector<double> priorities;
    // Read symbols of the type one by one
    for (char c : vehType) {
        priorities.push_back(static_cast<double>(c - '0'));
    }

    return priorities;

    // return std::vector<double>({3, 2, 1, 4});
}

void waitForCompletion() {
    std::vector<bool> finished(NUM_AGENTS, false);
    int numToFinish = NUM_AGENTS;

    while (numToFinish > 0) {
        MPI_Status status;
        MPI_Probe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);

        int agent = status.MPI_SOURCE - 1;  // Adjust index
        if (status.MPI_TAG == static_cast<int>(MessageTags::processFinished)) {
            int d;
            MPI_Recv(&d, 1, MPI_INT, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            if (!finished[agent]) {
                finished[agent] = true;
                numToFinish--;
            }
        } else if (status.MPI_TAG == static_cast<int>(MessageTags::processActive)) {
            int d;
            MPI_Recv(&d, 1, MPI_INT, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            if (finished[agent]) {
                finished[agent] = false;
                numToFinish++;
            }
        }
    }

    for (int a = 0; a < NUM_AGENTS; ++a) {
        int dump = 0;
        MPI_Bsend(&dump, 1, MPI_INT, a + 1, static_cast<int>(MessageTags::processFinished), MPI_COMM_WORLD);
    }
}

void proactiveReroute(const Graph &originalGraph, const std::vector<std::string> &departed, int simulationStep) {
    // Send newly started vehicles to their respective ACUs
    // Create vectors for each agent
    std::vector<std::vector<double>> agentData(NUM_AGENTS);

    // Pack data for each vehicle
    for (const std::string &veh : departed) {
        std::vector<std::string> defaultRoute = libtraci::Vehicle::getRoute(veh);

        int startEdge = std::stoi(*defaultRoute.begin());
        int selectedCluster = -1;

        // Check if the edge is a border edge
        if (originalGraph.getEdge(startEdge).isBorderEdge()) {
            // Select the appropriate cluster by checking which cluster the outgoing
            // edges belong to
            for (int nextEdge : originalGraph.getEdge(startEdge).getOutgoingEdges()) {
                if (!originalGraph.getEdge(nextEdge).isBorderEdge()) {
                    selectedCluster = originalGraph.getEdge(nextEdge).getClusterBitmask();
                }
            }
        } else {
            selectedCluster = originalGraph.getEdge(startEdge).getClusterBitmask();
        }
        // Pack vehicle data: [vehID, startEdge, endEdge, numPriorities,
        // priorities...]
        std::vector<double> vehData;
        vehData.push_back(std::stof(veh));
        vehData.push_back(static_cast<double>(defaultRoute.size()));
        for (const std::string &edge : defaultRoute) {
            vehData.push_back(std::stof(edge));
        }
        auto priorities = generatePriorities(veh);
        vehData.push_back(static_cast<double>(priorities.size()));
        vehData.insert(vehData.end(), priorities.begin(), priorities.end());

        if (selectedCluster == -1) {
            logError("type=NO_CLUSTER|vehicle=" + veh);
            continue;
        }

        // Add to relevant agents' data
        for (int n = 0; n < NUM_AGENTS; ++n) {
            if (selectedCluster & (1 << n)) {
                if (agentData[n].empty()) {
                    agentData[n].push_back(0);  // Reserve space for count
                }
                agentData[n].insert(agentData[n].end(), vehData.begin(), vehData.end());
                agentData[n][0] += 1;  // Increment vehicle count
            }
        }
    }

    // Send packed data to agents
    for (int n = 0; n < NUM_AGENTS; ++n) {
        if (!agentData[n].empty()) {
            agentData[n].insert(agentData[n].begin(), static_cast<double>(simulationStep));
            MPI_Bsend(agentData[n].data(), static_cast<int>(agentData[n].size()), MPI_DOUBLE, n + 1,
                      static_cast<int>(MessageTags::proactive), MPI_COMM_WORLD);
        } else {
            double f = static_cast<double>(simulationStep);
            MPI_Bsend(&f, 1, MPI_DOUBLE, n + 1, static_cast<int>(MessageTags::proactive), MPI_COMM_WORLD);
        }
    }

    logMessage("request|type=PROACTIVE|count=" + std::to_string(departed.size()));

    waitForCompletion();

    // Receive and process routes as before
    std::vector<std::vector<routeData>> newRoutes(NUM_AGENTS);
    for (int n = 0; n < NUM_AGENTS; ++n) {
        // Receive the combined message
        MPI_Status status;
        MPI_Probe(n + 1, static_cast<int>(MessageTags::proactive), MPI_COMM_WORLD, &status);
        int count;
        MPI_Get_count(&status, MPI_INT, &count);
        std::vector<double> receivedData(count);
        MPI_Recv(receivedData.data(), count, MPI_DOUBLE, n + 1, static_cast<int>(MessageTags::proactive),
                 MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        // Read the combined message
        int numVehicles = static_cast<int>(receivedData[0]);
        int pos = 1;
        for (int i = 0; i < numVehicles; ++i) {
            routeData rd;
            rd.vehicleID = static_cast<int>(receivedData[pos++]);
            int routeSize = static_cast<int>(receivedData[pos++]);
            rd.route.reserve(routeSize);
            for (int j = 0; j < routeSize; ++j) {
                rd.route.push_back(static_cast<int>(receivedData[pos++]));
            }
            newRoutes[n].push_back(rd);
        }
    }

    // Set routes in SUMO
    for (int n = 0; n < NUM_AGENTS; ++n) {
        for (const auto &rd : newRoutes[n]) {
            std::vector<std::string> route;
            route.reserve(rd.route.size());
            for (int e : rd.route) {
                route.push_back(std::to_string(e));
            }
            if (!route.empty()) {
                libtraci::Vehicle::setRoute(std::to_string(rd.vehicleID), route);
            } else {
                logError("type=NO_ROUTE|vehicle=" + std::to_string(rd.vehicleID));
            }
        }
    }
}

void reactiveReroute(int simulationStep) {
    // Send requests for reactive rerouting
    for (int n = 0; n < NUM_AGENTS; ++n) {
        MPI_Bsend(&simulationStep, 1, MPI_INT, n + 1, static_cast<int>(MessageTags::reactive), MPI_COMM_WORLD);
    }
    logMessage("request|type=REACTIVE|step=" + std::to_string(simulationStep));
    waitForCompletion();

    // Receive rerouted vehicles data
    for (int n = 0; n < NUM_AGENTS; ++n) {
        MPI_Status status;
        MPI_Probe(n + 1, static_cast<int>(MessageTags::reactive), MPI_COMM_WORLD, &status);
        int count;
        MPI_Get_count(&status, MPI_INT, &count);

        std::vector<routeData> toReroute;
        std::vector<int> receivedData(count);
        MPI_Recv(receivedData.data(), count, MPI_INT, n + 1, static_cast<int>(MessageTags::reactive), MPI_COMM_WORLD,
                 MPI_STATUS_IGNORE);

        // Process received routes
        int pos = 0;
        while (pos < count) {
            int vehID = receivedData[pos++];
            int routeSize = receivedData[pos++];
            std::vector<std::string> route;
            route.reserve(routeSize);
            for (int i = 0; i < routeSize; i++) {
                route.push_back(std::to_string(receivedData[pos++]));
            }
            if (!route.empty()) {
                libtraci::Vehicle::setRoute(std::to_string(vehID), route);
            } else {
                logError("type=NO_ROUTE|vehicle=" + std::to_string(vehID));
            }
        }
    }
}

void synchronizePositions(const Graph &originalGraph, const std::unordered_set<int> &runningVehicles,
                          std::unordered_map<int, std::unordered_map<int, double>> &currentDensities,
                          int simulationStep) {
    std::vector<std::vector<int>> clusterData(NUM_AGENTS);

    for (int veh : runningVehicles) {
        int index = libtraci::Vehicle::getRouteIndex(std::to_string(veh));
        auto curr = libtraci::Vehicle::getRoute(std::to_string(veh))[index];
        int currentEdge = std::stoi(curr);
        int selectedClusterBitmask = -1;

        if (originalGraph.getEdge(currentEdge).isBorderEdge()) {
            // Select the appropriate cluster by checking which cluster the outgoing
            // edges belong to
            for (int nextEdge : originalGraph.getEdge(currentEdge).getOutgoingEdges()) {
                if (!originalGraph.getEdge(nextEdge).isBorderEdge()) {
                    selectedClusterBitmask = originalGraph.getEdge(nextEdge).getClusterBitmask();
                }
            }
        } else {
            selectedClusterBitmask = originalGraph.getEdge(currentEdge).getClusterBitmask();
        }

        if (selectedClusterBitmask == -1) {
            logError("type=NO_CLUSTER|edge=" + std::to_string(currentEdge));
            continue;
        }

        for (int n = 0; n < NUM_AGENTS; ++n) {
            if (selectedClusterBitmask & (1 << n)) {
                clusterData[n].push_back(veh);
                clusterData[n].push_back(currentEdge);
            }
        }
    }

    // Send packed data to each ACU in a single message
    for (int n = 0; n < NUM_AGENTS; ++n) {
        if (!clusterData[n].empty()) {
            clusterData[n].insert(clusterData[n].begin(), simulationStep);
            MPI_Bsend(clusterData[n].data(), static_cast<int>(clusterData[n].size()), MPI_INT, n + 1,
                      static_cast<int>(MessageTags::synchronize), MPI_COMM_WORLD);
        } else {
            std::vector<int> data;
            data.push_back(simulationStep);
            MPI_Bsend(data.data(), static_cast<int>(data.size()), MPI_INT, n + 1,
                      static_cast<int>(MessageTags::synchronize), MPI_COMM_WORLD);
        }
    }
    logMessage("request|type=SYNCHRONIZATION|step=" + std::to_string(simulationStep));
    // Send densities
    for (int i = 0; i < NUM_AGENTS; ++i) {
        std::vector<double> combinedData;
        combinedData.push_back(static_cast<double>(simulationStep));
        combinedData.push_back(static_cast<double>(currentDensities[i].size()));  // Number of edges

        // Pack edge IDs and densities alternately
        for (auto &[fst, snd] : currentDensities[i]) {
            combinedData.push_back(static_cast<double>(fst));  // edge ID
            combinedData.push_back(snd);                       // density
        }

        MPI_Bsend(combinedData.data(), static_cast<int>(combinedData.size()), MPI_DOUBLE, i + 1,
                  static_cast<int>(MessageTags::synchronize), MPI_COMM_WORLD);
    }
}

void masterProcess(const Graph &originalGraph, const std::string &scenarioPath, const std::string &tripInfoFileName,
                   const std::string &edgeDataFileName, const std::string &vehRoutesFileName, int syncIntervalLength,
                   int simulationEnd) {
    // Main process that will control SUMO
    int reroutingInterval = 60;
    // Frequent synchronization with SUMO for purposes of reactive rerouting
    int synchronizationInterval = syncIntervalLength;

    int simulationStep = 0;

    // Construct paths based on scenarioPath
    std::string configPath = scenarioPath + "/scenario.sumocfg";
    // Use the provided filename in the current directory
    auto outputPath = [&scenarioPath](const std::string &name, const std::string &fallback) {
        std::filesystem::path path = name.empty() ? fallback : name;
        return (path.is_absolute() ? path : std::filesystem::path(scenarioPath) / path).string();
    };
    std::string tripInfoPath = outputPath(tripInfoFileName, "tripinfo.xml");
    std::string edgeDataPath = outputPath(edgeDataFileName, "edgedata.xml");
    std::string vehRoutesPath = outputPath(vehRoutesFileName, "vehroutes.xml");

    // Start SUMO simulation
    libtraci::Simulation::start({"sumo", "-c", configPath, "--tripinfo-output", tripInfoPath, "--edgedata-output",
                                 edgeDataPath, "--vehroute-output", vehRoutesPath,
                                 "--vehroute-output.write-unfinished", "true", "--start"});

    int numEdges = originalGraph.getNumEdges();

    std::vector<int> vehicleCounts(numEdges, 0);

    std::unordered_set<int> runningVehicles;
    std::unordered_map<int, std::unordered_map<int, double>> currentDensities;
    for (int e = 0; e < numEdges; ++e) {
        for (int i = 0; i < NUM_AGENTS; ++i) {
            if (originalGraph.getEdge(e).inCluster(i)) {
                currentDensities[i][e] = 0;
            }
        }
    }

    // Run simulation until either the end time is reached (if specified) or no more vehicles are expected
    while (libtraci::Simulation::getMinExpectedNumber() && (simulationEnd < 0 || simulationStep < simulationEnd)) {
        libtraci::Simulation::step();
        simulationStep += 1;

        std::cout << "LOG|MASTER|" << "step=" + std::to_string(simulationStep) << std::endl;

        for (int e = 0; e < numEdges; ++e) {
            vehicleCounts[e] += libtraci::Edge::getLastStepVehicleNumber(std::to_string(e));
        }

        // Update list of running vehicles
        std::vector<std::string> departedVehicles = libtraci::Simulation::getDepartedIDList();

        for (const std::string &veh : departedVehicles) {
            runningVehicles.insert(std::stoi(veh));
        }
        for (const std::string &veh : libtraci::Simulation::getArrivedIDList()) {
            runningVehicles.erase(std::stoi(veh));
        }

        // PROACTIVE REROUTING OF NEW VEHICLES
        if (!departedVehicles.empty()) {
            proactiveReroute(originalGraph, departedVehicles, simulationStep);
        }

        // SYNCHRONIZATION OF ACUS WITH SIMULATION
        if (simulationStep % synchronizationInterval == 0) {
            // Synchronize positions of vehicle in the simulation with the ones in the ACUs
            for (auto &[clID, p] : currentDensities) {
                for (auto &[edge, density] : p) {
                    const double length = originalGraph.getEdge(edge).getLength();
                    const double maxDensity = originalGraph.getEdge(edge).getMaxDensity();
                    double relativeDensity = 0;
                    if (std::isfinite(length) && length > 0 && std::isfinite(maxDensity) && maxDensity > 0) {
                        relativeDensity = 1000 * (static_cast<double>(vehicleCounts[edge]) / reroutingInterval) /
                                          length / maxDensity;
                    } else {
                        logError("Invalid density geometry for edge " + std::to_string(edge));
                    }
                    density = std::isfinite(relativeDensity) ? std::clamp(relativeDensity, 0.0, 1.0) : 0.0;
                    vehicleCounts[edge] = 0;
                }
            }
            synchronizePositions(originalGraph, runningVehicles, currentDensities, simulationStep);
        }

        // REACTIVE REROUTING OF CONGESTIONS
        if (simulationStep % reroutingInterval == 0) {
            reactiveReroute(simulationStep);
        }
    }

    for (int n = 0; n < NUM_AGENTS; ++n) {
        int dump = 0;
        MPI_Bsend(&dump, 1, MPI_INT, n + 1, static_cast<int>(MessageTags::exit), MPI_COMM_WORLD);
    }

    libtraci::Simulation::close();
}

// ACU process function
void acuProcess(Router &router) {
    int worldRank;  // from 1 to 8
    MPI_Comm_rank(MPI_COMM_WORLD, &worldRank);
    bool operationExit = false;
    int dump;
    // ACUs main operation involves waiting for messages and processing them in
    // order of arrival
    while (!operationExit) {
        // Check for an incoming message
        MPI_Status status;
        MPI_Probe(0, MPI_ANY_TAG, MPI_COMM_WORLD, &status);

        if (status.MPI_TAG == static_cast<int>(MessageTags::proactive)) {
            // Receive the combined data vector
            int count;
            MPI_Get_count(&status, MPI_DOUBLE, &count);
            std::vector<double> receivedData(count);
            MPI_Recv(receivedData.data(), count, MPI_DOUBLE, 0, static_cast<int>(MessageTags::proactive),
                     MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            // Extract vehicle data
            std::vector<std::tuple<int, std::vector<int>, std::vector<double>>> toRoute;
            int currentStep = static_cast<int>(receivedData[0]);
            if (count > 1) {
                int numVehicles = static_cast<int>(receivedData[1]);
                int currentPos = 2;

                for (int i = 0; i < numVehicles; i++) {
                    int vehID = static_cast<int>(receivedData[currentPos++]);
                    int numEdges = static_cast<int>(receivedData[currentPos++]);
                    std::vector<int> route;
                    route.reserve(numEdges);
                    for (int j = 0; j < numEdges; j++) {
                        route.push_back(static_cast<int>(receivedData[currentPos++]));
                    }
                    int numPriorities = static_cast<int>(receivedData[currentPos++]);

                    std::vector<double> priorities;
                    priorities.reserve(numPriorities);
                    for (int p = 0; p < numPriorities; p++) {
                        priorities.push_back(receivedData[currentPos++]);
                    }

                    toRoute.emplace_back(vehID, route, priorities);
                }
            }

            // Process routes and send back results
            std::vector<routeData> newRoutes = router.proactivelyRoute(toRoute, currentStep);

            // Create a single vector to be sent back
            std::vector<double> combinedData;
            combinedData.push_back(static_cast<double>(newRoutes.size()));
            for (auto &rd : newRoutes) {
                combinedData.push_back(static_cast<double>(rd.vehicleID));
                combinedData.push_back(static_cast<double>(rd.route.size()));
                for (int e : rd.route) {
                    combinedData.push_back(static_cast<double>(e));
                }
            }
            MPI_Bsend(combinedData.data(), static_cast<int>(combinedData.size()), MPI_DOUBLE, 0,
                      static_cast<int>(MessageTags::proactive), MPI_COMM_WORLD);
        } else if (status.MPI_TAG == static_cast<int>(MessageTags::synchronize)) {
            int size;
            MPI_Get_count(&status, MPI_INT, &size);

            std::vector<int> receivedData(size);
            MPI_Recv(receivedData.data(), size, MPI_INT, 0, static_cast<int>(MessageTags::synchronize), MPI_COMM_WORLD,
                     MPI_STATUS_IGNORE);
            int currentStep = receivedData[0];
            std::vector<positionData> positions;
            positions.reserve((size - 1) / 2);
            for (int i = 1; i < size; i += 2) {
                positions.emplace_back(receivedData[i], receivedData[i + 1]);
            }

            int count;
            MPI_Status densityStatus;
            MPI_Probe(0, static_cast<int>(MessageTags::synchronize), MPI_COMM_WORLD, &densityStatus);
            MPI_Get_count(&densityStatus, MPI_DOUBLE, &count);
            std::vector<double> receivedDensities(count);
            MPI_Recv(receivedDensities.data(), count, MPI_DOUBLE, 0, static_cast<int>(MessageTags::synchronize),
                     MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            // Extract edge densities from combined data
            int numEdges = static_cast<int>(receivedDensities[1]);
            std::unordered_map<int, double> currentDensities;
            for (int i = 1; i < numEdges + 1; i++) {
                int edgeID = static_cast<int>(receivedDensities[2 * i]);
                double density = receivedDensities[2 * i + 1];
                currentDensities[edgeID] = density;
            }

            router.synchronizePositions(positions, currentDensities, currentStep);
        } else if (status.MPI_TAG == static_cast<int>(MessageTags::reactive)) {
            int currentStep;
            MPI_Recv(&currentStep, 1, MPI_INT, 0, static_cast<int>(MessageTags::reactive), MPI_COMM_WORLD,
                     MPI_STATUS_IGNORE);

            // Process rerouting and get new routes
            std::vector<routeData> newRoutes = router.reactivelyReroute(currentStep);

            // Pack and send route data
            std::vector<int> responseData;
            for (const auto &rd : newRoutes) {
                responseData.push_back(rd.vehicleID);
                responseData.push_back(static_cast<int>(rd.route.size()));
                responseData.insert(responseData.end(), rd.route.begin(), rd.route.end());
            }

            MPI_Bsend(responseData.data(), static_cast<int>(responseData.size()), MPI_INT, 0,
                      static_cast<int>(MessageTags::reactive), MPI_COMM_WORLD);
        } else if (status.MPI_TAG == static_cast<int>(MessageTags::exit)) {
            MPI_Recv(&dump, 1, MPI_INT, 0, static_cast<int>(MessageTags::exit), MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            logMessage("exit|id=" + std::to_string(worldRank));
            operationExit = true;
        }
    }
}

int main(int argc, char *argv[]) {
    // Default values
    int numAgents = 0;
    std::string scenarioPath = "";
    int synchronizationInterval = 0;
    std::string tripInfoFileName = "";
    std::string edgeDataFileName = "";
    std::string vehRoutesFileName = "";  // Added vehroutes filename
    std::string logLevel = "info";
    int simulationEnd = -1;       // Negative value indicates no end time specified
    double epsilon = 1.0;         // Default epsilon value
    int K = 1;                    // Default K value
    double reroutePortion = 1.0;  // Default reroute portion

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --agents <num>        Number of agents (required)\n"
                      << "  --path <path>         Absolute path to scenario root folder (required)\n"
                      << "  --sync <interval>     Synchronization interval (required)\n"
                      << "  --tripinfo <name>     Name of trip info file (optional, defaults to tripinfo.xml)\n"
                      << "  --edgedata <name>     Name of edge data file (optional, defaults to edgedata.xml)\n"
                      << "  --vehroutes <name>    Name of vehicle routes file (optional, defaults to vehroutes.xml)\n"
                      << "  --end <steps>         Simulation end time in steps (optional, if not specified runs until "
                         "completion)\n"
                      << "  --epsilon <value>     Epsilon value for MLS search (optional, must be between 0 and 1, "
                         "defaults to 1.0)\n"
                      << "  --k <value>           K value for hierarchical search (optional, defaults to 1)\n"
                      << "  --reroute <value>     Portion of vehicles to reroute when congestion is detected "
                         "(optional, must be between 0 and 1, defaults to 1.0)\n"
                      << "  --logging             Enable logging (optional)\n"
                      << "  --help, -h            Show this help message\n";
            return 0;
        } else if (arg == "--agents" && i + 1 < argc) {
            numAgents = std::stoi(argv[++i]);
            NUM_AGENTS = numAgents;
        } else if (arg == "--path" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (arg == "--sync" && i + 1 < argc) {
            synchronizationInterval = std::stoi(argv[++i]);
        } else if (arg == "--tripinfo" && i + 1 < argc) {
            tripInfoFileName = argv[++i];
        } else if (arg == "--edgedata" && i + 1 < argc) {
            edgeDataFileName = argv[++i];
        } else if (arg == "--vehroutes" && i + 1 < argc) {
            vehRoutesFileName = argv[++i];
        } else if (arg == "--end" && i + 1 < argc) {
            simulationEnd = std::stoi(argv[++i]);
        } else if (arg == "--epsilon" && i + 1 < argc) {
            epsilon = std::stod(argv[++i]);
            if (epsilon < 0 || epsilon > 1) {
                logError("Epsilon value must be between 0 and 1");
                return 1;
            }
        } else if (arg == "--k" && i + 1 < argc) {
            K = std::stoi(argv[++i]);
            if (K < 1) {
                logError("K value must be at least 1");
                return 1;
            }
        } else if (arg == "--reroute" && i + 1 < argc) {
            reroutePortion = std::stod(argv[++i]);
            if (reroutePortion < 0 || reroutePortion > 1) {
                logError("Reroute portion must be between 0 and 1");
                return 1;
            }
        } else if (arg == "--logging") {
            ENABLE_LOGGING = true;
        }
    }

    // Validate required parameters
    if (numAgents <= 0 || scenarioPath.empty() || synchronizationInterval <= 0) {
        logError("Missing required parameters. Use --help for usage information.");
        return 1;
    }
    // Set up paths
    std::string nodePath = scenarioPath + "/graphs/nodes" + std::to_string(numAgents) + ".csv";
    std::string edgePath = scenarioPath + "/graphs/edges" + std::to_string(numAgents) + ".csv";

    // Read network data
    auto [originalNodes, originalEdges] = readSumoNetwork(nodePath, edgePath);
    Graph originalGraph(originalNodes, originalEdges);

    // Initialize MPI
    MPI_Init(&argc, &argv);
    int worldRank;
    MPI_Comm_rank(MPI_COMM_WORLD, &worldRank);
    int worldSize;
    MPI_Comm_size(MPI_COMM_WORLD, &worldSize);
    if (worldSize != NUM_AGENTS + 1) {
        if (worldRank == 0) {
            logError("Expected " + std::to_string(NUM_AGENTS + 1) +
                     " MPI processes (one master plus --agents workers).");
        }
        MPI_Finalize();
        return 1;
    }

    // Set up MPI buffer
    int bufferSize = 1073741824;
    std::vector<char> buff(bufferSize);
    MPI_Buffer_attach(buff.data(), bufferSize);

    try {
        if (worldRank == 0) {
            masterProcess(originalGraph, scenarioPath, tripInfoFileName, edgeDataFileName, vehRoutesFileName,
                          synchronizationInterval, simulationEnd);
        } else if (worldRank > 0 && worldRank <= NUM_AGENTS) {
            // ACU processes
            // Create Traffic estimator
            TrafficEstimator t(scenarioPath + "/models/log/density_60.json");
            SpeedEstimator s(scenarioPath + "/models/log/speed_60.json");
            FuelEstimator f(scenarioPath + "/models/log/fuel_60.json");

            Router clusterRouter(worldRank - 1, NUM_AGENTS, 60, originalGraph, K, t, s, f);
            clusterRouter.setEpsilon(epsilon);
            clusterRouter.setReroutePortion(reroutePortion);
            clusterRouter.setLogging(ENABLE_LOGGING);
            acuProcess(clusterRouter);
        }

    } catch (const std::exception &error) {
        logError("MPI rank " + std::to_string(worldRank) + ": " + error.what());
        // A single failed worker must not leave the master or peers waiting forever.
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    void *detachedBuffer;
    int detachedSize;
    MPI_Buffer_detach(&detachedBuffer, &detachedSize);
    MPI_Finalize();
    return 0;
}
