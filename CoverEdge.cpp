/**
 * @file CoverEdge.cpp
 * @brief Implementation of CoverEdge methods for hierarchical cover edges.
 */
//
// Created by Tim on 20.03.2024.
//

#include "CoverEdge.h"

#include "Edge.h"

CoverEdge::CoverEdge(const CoverEdge &firstEdge, const CoverEdge &secondEdge) {
    this->insideNodes = firstEdge.getInsideNodes();
    this->insideNodes.insert(insideNodes.end(), secondEdge.getInsideNodes().begin() + 1,
                             secondEdge.getInsideNodes().end());

    this->insideEdges = firstEdge.getInsideEdges();
    this->insideEdges.insert(insideEdges.end(), secondEdge.getInsideEdges().begin(), secondEdge.getInsideEdges().end());
    this->compositeCoverEdges = std::make_pair(firstEdge.getId(), secondEdge.getId());

    this->length = firstEdge.getLength() + secondEdge.getLength();
    this->dangerFactor =
        (firstEdge.getDangerFactor() * firstEdge.getNumHops()) +
        (secondEdge.getDangerFactor() * secondEdge.getNumHops()) / (firstEdge.getNumHops() + secondEdge.getNumHops());
    this->speedLimit = (firstEdge.getSpeedLimit() + secondEdge.getSpeedLimit()) / 2;
    this->numLanes = (firstEdge.getNumLanes() + secondEdge.getNumLanes()) / 2;
    this->maxThroughput = (firstEdge.getMaxThroughput() + secondEdge.getMaxThroughput()) / 2;
    this->clusterBitmask = firstEdge.getClusterBitmask() | secondEdge.getClusterBitmask();
}

CoverEdge::CoverEdge(const Edge &originalEdge) {
    this->insideNodes = std::vector({originalEdge.getStartNode(), originalEdge.getEndNode()});
    this->insideEdges = std::vector({originalEdge.getID()});
    this->id = originalEdge.getID();
    this->clusterBitmask = originalEdge.getClusterBitmask();
    this->length = originalEdge.getLength();
    this->dangerFactor = originalEdge.getDangerFactor();
    this->speedLimit = originalEdge.getSpeedLimit();
    this->numLanes = static_cast<double>(originalEdge.getNumLanes());
    this->maxThroughput = (this->numLanes * this->speedLimit) / this->length;
}

CoverEdge::CoverEdge(const CoverEdge &coverEdge) {
    // Copy constructor
    this->insideNodes = std::vector(coverEdge.getInsideNodes());
    this->insideEdges = std::vector(coverEdge.getInsideEdges());
    this->compositeCoverEdges = {coverEdge.getId(), -1};
    this->length = coverEdge.getLength();
    this->dangerFactor = coverEdge.getDangerFactor();
    this->speedLimit = coverEdge.getSpeedLimit();
    this->numLanes = coverEdge.getNumLanes();
    this->maxThroughput = coverEdge.getMaxThroughput();
    this->id = coverEdge.getId();
    this->clusterBitmask = coverEdge.getClusterBitmask();
}

CoverEdge::CoverEdge(int id, int clusterBitmask, const std::vector<int> &insideEdges) {
    // Constructor for empty edge with id from another cluster
    this->id = id;
    this->length = -1;
    this->dangerFactor = -1;
    this->speedLimit = -1;
    this->insideEdges = std::vector<int>(insideEdges);
    this->numLanes = -1;
    this->maxThroughput = -1;
    this->clusterBitmask = clusterBitmask;
}

const std::vector<int> &CoverEdge::getInsideNodes() const {
    return insideNodes;
}

int CoverEdge::getStartNode() const {
    return insideNodes.front();
}

int CoverEdge::getEndNode() const {
    return insideNodes.back();
}

const std::vector<int> &CoverEdge::getInsideEdges() const {
    return insideEdges;
}

int CoverEdge::getStartEdge() const {
    return insideEdges.front();
}

int CoverEdge::getEndEdge() const {
    return insideEdges.back();
}

int CoverEdge::getId() const {
    return id;
}

std::pair<int, int> CoverEdge::getCompositeCoverEdges() const {
    return compositeCoverEdges;
}

double CoverEdge::getLength() const {
    return length;
}

double CoverEdge::getSpeedLimit() const {
    return speedLimit;
}

double CoverEdge::getNumLanes() const {
    return numLanes;
}

double CoverEdge::getMaxThroughput() const {
    return maxThroughput;
}

double CoverEdge::getDangerFactor() const {
    return dangerFactor;
}

std::vector<double> CoverEdge::getBuildingParameters() const {
    // return std::vector<double>({length, speedLimit, numLanes, dangerFactor});
    return std::vector<double>({length, speedLimit, numLanes, static_cast<double>(insideEdges.size())});
}

void CoverEdge::setId(int newID) {
    this->id = newID;
}

int CoverEdge::getClusterBitmask() const {
    return clusterBitmask;
}

void CoverEdge::setClusterBitmask(int bitmask) {
    clusterBitmask = bitmask;
}

bool CoverEdge::inCluster(int clusterID) const {
    return clusterBitmask & (1 << clusterID);
}

int CoverEdge::getNumHops() const {
    return static_cast<int>(insideEdges.size());
}
