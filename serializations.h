
#ifndef MASHIERARCHICALSYSTEM_SERIALIZATIONS_H
#define MASHIERARCHICALSYSTEM_SERIALIZATIONS_H

#include <cstdlib>
#include <utility>
#include <vector>

/**
 * @enum MessageTags
 * @brief MPI message tags used for inter-agent communication.
 */
enum class MessageTags : short {
    proactive = 0,        ///< Proactive routing requests
    reactive = 1,         ///< Reactive rerouting requests
    synchronize = 2,      ///< Synchronization message
    routeData = 3,        ///< Route data payload
    demandData = 4,       ///< Demand data payload
    continueRoute = 5,    ///< Continue hierarchical search
    finishedRoute = 6,    ///< Completed route available
    addOffset = 7,        ///< Internal tag: add offset
    processActive = 8,    ///< Agent is active
    processFinished = 9,  ///< Agent finished processing
    label = 10,           ///< Label transfer payload
    positionData = 11,    ///< Position data payload
    routeNotFound = 12,   ///< Route not found notification
    acceptedRoute = 13,   ///< Accepted route notification
    foundRoutes = 14,     ///< Candidate routes payload
    exit = 20             ///< Exit signal
};

/**
 * @struct positionData
 * @brief Current vehicle position used for synchronization.
 */
struct positionData {
    int vehicleID;  ///< Vehicle identifier
    int edgeID;     ///< Current edge identifier

    positionData() : vehicleID(-1), edgeID(-1) {
    }
    positionData(int id, int e) : vehicleID(id), edgeID(e) {
    }
};

/**
 * @struct routeData
 * @brief Computed route for a vehicle.
 */
struct routeData {
    int vehicleID;           ///< Vehicle identifier
    std::vector<int> route;  // List of edge IDs in the new route

    routeData() : vehicleID(-1), route(std::vector<int>()) {
    }
    routeData(int veh, std::vector<int> r) : vehicleID(veh), route(std::move(r)) {
    }
};

/**
 * @struct demandData
 * @brief Routing request describing a start and goal with priorities.
 */
struct demandData {
    int vehicleID;                   ///< Vehicle identifier
    int startEdge;                   ///< Start edge ID
    int endEdge;                     ///< Goal edge ID
    std::vector<double> priorities;  ///< Criteria weights/priorities

    demandData() : vehicleID(-1), startEdge(-1), endEdge(-1), priorities(std::vector<double>()) {
    }
    demandData(int v, int s, int e, std::vector<double> priorities)
        : vehicleID(v), startEdge(s), endEdge(e), priorities(std::move(priorities)) {
    }
};

#endif  // MASHIERARCHICALSYSTEM_SERIALIZATIONS_H
