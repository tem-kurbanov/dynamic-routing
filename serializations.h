
#ifndef MASHIERARCHICALSYSTEM_SERIALIZATIONS_H
#define MASHIERARCHICALSYSTEM_SERIALIZATIONS_H

#include <cstdlib>
#include <utility>

enum class MessageTags : short {
    proactive = 0,
    reactive = 1,
    synchronize = 2,
    routeData = 3,
    demandData = 4,
    continueRoute = 5,
    finishedRoute = 6,
    addOffset = 7,
    processActive = 8,
    processFinished = 9,
    label = 10,
    positionData = 11,
    routeNotFound = 12,
    acceptedRoute = 13,
    foundRoutes = 14,
    exit = 20
};

struct positionData {
    int vehicleID;
    int edgeID;

    positionData() : vehicleID(-1), edgeID(-1) {
    }
    positionData(int id, int e) : vehicleID(id), edgeID(e) {
    }
};

struct routeData {
    int vehicleID;
    std::vector<int> route;  // List of edge IDs in the new route

    routeData() : vehicleID(-1), route(std::vector<int>()) {
    }
    routeData(int veh, std::vector<int> r) : vehicleID(veh), route(std::move(r)) {
    }
};

struct demandData {
    int vehicleID;
    int startEdge;
    int endEdge;
    std::vector<double> priorities;

    demandData() : vehicleID(-1), startEdge(-1), endEdge(-1), priorities(std::vector<double>()) {
    }
    demandData(int v, int s, int e, std::vector<double> priorities)
        : vehicleID(v), startEdge(s), endEdge(e), priorities(std::move(priorities)) {
    }
};

#endif  // MASHIERARCHICALSYSTEM_SERIALIZATIONS_H
