#include "CoverGraph.h"

#include <algorithm>
#include <iostream>
#include <optional>
#include <set>
#include <stack>
#include <utility>
#include <vector>

#include "CoverEdge.h"
#include "Edge.h"
#include "Node.h"

// Move constructor
CoverGraph::CoverGraph(CoverGraph&& other) noexcept
    : precedingCoverGraph(other.precedingCoverGraph),  // Transfer the pointer
      numCoverNodes(other.numCoverNodes),
      numCoverEdges(other.numCoverEdges),
      coverNodes(std::move(other.coverNodes)),
      coverEdges(std::move(other.coverEdges)),
      outgoingCoverEdges(std::move(other.outgoingCoverEdges)),
      incomingCoverEdges(std::move(other.incomingCoverEdges)),
      originalGraph(other.getOriginalGraph()) {
    // Nullify the source's pointer to prevent double deletion.
    //    other.precedingCoverGraph = nullptr;

    // Reset other's primitive types if necessary. Since these are just sizes, resetting them to 0 makes sense.
    //    other.numCoverNodes = 0;
    //    other.numCoverEdges = 0;
}

// Copy constructor
CoverGraph::CoverGraph(const CoverGraph& other)
    : precedingCoverGraph(other.precedingCoverGraph),
      numCoverNodes(other.numCoverNodes),
      numCoverEdges(other.numCoverEdges),
      coverNodes(other.coverNodes),
      coverEdges(other.coverEdges),
      outgoingCoverEdges(other.outgoingCoverEdges),
      incomingCoverEdges(other.incomingCoverEdges),
      originalGraph(other.getOriginalGraph()) {
}

// Constructor for the kPC cover graph
CoverGraph::CoverGraph(const std::shared_ptr<CoverGraph>& precedingCoverGraph)
    : precedingCoverGraph(precedingCoverGraph),
      originalGraph(precedingCoverGraph->getOriginalGraph()),
      clusterID(precedingCoverGraph->getClusterID()) {
    kPathCover();
}
// Constructor for the initial cover graph
CoverGraph::CoverGraph(const Graph& graph)
    : originalGraph(graph),
      coverNodes(std::set<int>()),
      coverEdges(std::vector<CoverEdge>()),
      outgoingCoverEdges(std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>()),
      incomingCoverEdges(std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>()) {

    numCoverNodes = originalGraph.getNumNodes();
    clusterID = -1;

    for (int i = 0; i < numCoverNodes; ++i) {
        coverNodes.insert(i);
    }

    std::vector<Edge> originalEdges = originalGraph.getEdges();
    numCoverEdges = 0;
    for (size_t i = 0; i < originalEdges.size(); ++i) {
        CoverEdge ce(originalEdges[i]);
        ce.setId(numCoverEdges);

        originalToCoverEdges[originalEdges[i].getID()] = numCoverEdges;

        int startNode = originalEdges[i].getStartNode();
        int endNode = originalEdges[i].getEndNode();
        outgoingCoverEdges[startNode][static_cast<int>(i)].insert(numCoverEdges);
        incomingCoverEdges[endNode][static_cast<int>(i)].insert(numCoverEdges);

        numCoverEdges += 1;
        this->coverEdges.push_back(ce);
    }
}

// void CoverGraph::updateClusterInfo(const std::unordered_map<int, double>& densities, const std::unordered_map<int,
// double>& speeds, const std::unordered_map<int, double>& fuels, std::optional<int> clusterOpt) {
void CoverGraph::updateClusterID(int clusterOpt) {
    // For ground cover, exclude non-cluster edges and update densities of cluster edges
    if (precedingCoverGraph == nullptr) {
        /*
         * Go through every edge
         * If it is in the cluster, save it and assign it a new id
         * Update inner containers with new edges
         */
        clusterID = clusterOpt;
        std::vector<CoverEdge> newEdges;
        std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> newOutgoing;
        std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> newIncoming;
        int numEdges = 0;
        originalToCoverEdges.clear();
        for (int i = 0; i < numCoverEdges; ++i) {
            if (coverEdges[i].getClusterBitmask() & (1 << clusterID)) {
                // The edge is in the cluster, save it
                int originalId = coverEdges[i].getStartEdge();
                originalToCoverEdges[originalId] = numEdges;
                coverEdges[i].setId(numEdges);

                int startNode = coverEdges[i].getStartNode();
                int endNode = coverEdges[i].getEndNode();
                newOutgoing[startNode][i].insert(numEdges);
                newIncoming[endNode][i].insert(numEdges);

                newEdges.push_back(coverEdges[i]);
                numEdges += 1;
            }
        }
        // Replace old containers with new ones
        coverEdges = newEdges;
        outgoingCoverEdges = newOutgoing;
        incomingCoverEdges = newIncoming;
        numCoverEdges = numEdges;
    }
}

void CoverGraph::kPathCover() {
    buildCoverNodes();
    buildCoverEdges();
}

void CoverGraph::buildCoverNodes() {
    std::unordered_map<int, bool> inCover;
    numCoverNodes = 0;
    numCoverEdges = 0;

    // LR-deg approach
    // Sort nodes by degree from lowest to highest
    // If a vertex is not in the cover, add all neighbor vertices to the cover

    // Sort nodes by degree
    std::vector<std::pair<int, int>> nodeDegrees;
    for (int n : precedingCoverGraph->coverNodes) {
        inCover[n] = false;
        int degree = 0;
        if (precedingCoverGraph->hasOutgoingEdges(n)) {
            auto outEdges = precedingCoverGraph->getOutgoingEdges(n);
            for (const auto& pair : outEdges) {
                degree += static_cast<int>(pair.second.size());
            }
        }

        if (precedingCoverGraph->hasIncomingEdges(n)) {
            auto inEdges = precedingCoverGraph->getIncomingEdges(n);
            for (const auto& pair : inEdges) {
                degree += static_cast<int>(pair.second.size());
            }
        }

        nodeDegrees.emplace_back(n, degree);
    }
    sort(nodeDegrees.begin(), nodeDegrees.end(),
         [](std::pair<int, int> a, std::pair<int, int> b) { return a.second < b.second; });

    // Pick KPC nodes
    int numProcessed = 0;
    for (auto& node : nodeDegrees) {
        int n = node.first;
        if (originalGraph.getNode(n).isBorderNode()) {
            // Leave the node in the cover
            inCover[n] = true;
            coverNodes.insert(n);
            numCoverNodes += 1;
            continue;
        }

        numProcessed += 1;
        if (!inCover[n]) {
            if (precedingCoverGraph->hasOutgoingEdges(n)) {
                for (const auto& it : precedingCoverGraph->getOutgoingEdges(n)) {
                    for (int edge : it.second) {
                        int nextNode = precedingCoverGraph->getEdgeNodes(edge).back();
                        if (!inCover[nextNode]) {
                            inCover[nextNode] = true;
                            coverNodes.insert(nextNode);
                            numCoverNodes += 1;
                        }
                    }
                }
            }
            if (precedingCoverGraph->hasIncomingEdges(n)) {
                for (const auto& it : precedingCoverGraph->getIncomingEdges(n)) {
                    for (int edge : it.second) {
                        int nextNode = precedingCoverGraph->getEdgeNodes(edge).front();
                        if (!inCover[nextNode]) {
                            inCover[nextNode] = true;
                            coverNodes.insert(nextNode);
                            numCoverNodes += 1;
                        }
                    }
                }
            }
        }
    }

    //    std::cout << "Initial number of vertices -- " << precedingCoverGraph->getNumNodes() << "\n";
    //    std::cout << "Number of cover vertices -- " << numCoverNodes << "\n";
    //    std::cout << "Relative vertex percentage -- " << ((double)numCoverNodes /
    //    (double)precedingCoverGraph->getNumNodes()) * 100 << "\n";
}

void CoverGraph::buildCoverEdges() {
    unsigned long numInitCoverEdges = 0;
    unsigned long domPruned = 0;

    auto inCover = [this](int n) -> bool {
        // Check if the node belongs to the coverNodes set
        return coverNodes.count(n) > 0;
    };

    std::vector<bool> edgeProcessed(precedingCoverGraph->getNumEdges(), false);
    std::unordered_map<int, std::unordered_map<int, std::vector<CoverEdge>>> tempCoverEdges;
    // Copy edges that remain in the cover to the new cover graph
    for (int i = 0; i < precedingCoverGraph->getNumEdges(); ++i) {
        int startNode = precedingCoverGraph->getCoverEdge(i).getStartNode();
        int endNode = precedingCoverGraph->getCoverEdge(i).getEndNode();
        if (inCover(startNode) && inCover(endNode)) {
            int startEdge = precedingCoverGraph->getCoverEdge(i).getStartEdge();
            int endEdge = precedingCoverGraph->getCoverEdge(i).getEndEdge();
            tempCoverEdges[startEdge][endEdge].emplace_back(precedingCoverGraph->getCoverEdge(i));
            numInitCoverEdges += 1;
        }
    }

    // Build cover edges
    for (int n : precedingCoverGraph->getCoverNodes()) {
        if (inCover(n) || !precedingCoverGraph->hasIncomingEdges(n) || !precedingCoverGraph->hasOutgoingEdges(n))
            continue;
        std::unordered_map<int, std::unordered_set<int>> incomingEdges = precedingCoverGraph->getIncomingEdges(n);
        std::unordered_map<int, std::unordered_set<int>> outgoingEdges = precedingCoverGraph->getOutgoingEdges(n);

        for (const auto& it1 : incomingEdges) {
            for (int e1 : it1.second) {
                for (const auto& it2 : outgoingEdges) {
                    for (int e2 : it2.second) {
                        // Make sure there is a connection between the incoming and the outgoing edge
                        if (originalGraph.getEdge(precedingCoverGraph->getCoverEdge(e1).getEndEdge())
                                .getOutgoingEdges()
                                .count(precedingCoverGraph->getCoverEdge(e2).getStartEdge()) == 0) {
                            continue;
                        }

                        std::vector<double> newCriteria =
                            linkPaths(precedingCoverGraph->getCoverEdge(e1).getBuildingParameters(),
                                      precedingCoverGraph->getCoverEdge(e1).getNumHops(),
                                      precedingCoverGraph->getCoverEdge(e2).getBuildingParameters(),
                                      precedingCoverGraph->getCoverEdge(e2).getNumHops());
                        numInitCoverEdges += 1;
                        int startEdge = precedingCoverGraph->getCoverEdge(e1).getStartEdge();
                        int endEdge = precedingCoverGraph->getCoverEdge(e2).getEndEdge();

                        if (PRUNE_DOMINATED) {
                            // Compare to existing edges (domination pruning)
                            bool isDominated = false;
                            std::stack<int> toRemove;
                            for (int i = 0; i < tempCoverEdges[startEdge][endEdge].size(); ++i) {
                                if (checkDomination(tempCoverEdges[startEdge][endEdge][i].getBuildingParameters(),
                                                    newCriteria)) {
                                    isDominated = true;
                                    domPruned += 1;
                                    break;
                                }
                                if (checkDomination(newCriteria,
                                                    tempCoverEdges[startEdge][endEdge][i].getBuildingParameters())) {
                                    toRemove.push(i);
                                }
                            }
                            if (isDominated) {
                                continue;
                            }

                            while (!toRemove.empty()) {
                                int ind = toRemove.top();
                                toRemove.pop();
                                tempCoverEdges[startEdge][endEdge].erase(tempCoverEdges[startEdge][endEdge].begin() +
                                                                         ind);
                                domPruned += 1;
                            }
                        }

                        CoverEdge newEdge(precedingCoverGraph->getCoverEdge(e1), precedingCoverGraph->getCoverEdge(e2));
                        tempCoverEdges[startEdge][endEdge].emplace_back(newEdge);
                    }
                }
            }
        }
    }

    // Clear existing data in the structures
    numCoverEdges = 0;
    coverEdges.clear();
    outgoingCoverEdges.clear();
    incomingCoverEdges.clear();
    for (const auto& it : tempCoverEdges) {
        for (const auto& it2 : it.second) {
            for (auto edge : it2.second) {
                edge.setId(numCoverEdges);
                coverEdges.push_back(edge);
                outgoingCoverEdges[edge.getStartNode()][edge.getStartEdge()].insert(numCoverEdges);
                incomingCoverEdges[edge.getEndNode()][edge.getEndEdge()].insert(numCoverEdges);
                numCoverEdges += 1;
            }
        }
    }

    //    std::cout << "Initial number of edges -- " << precedingCoverGraph->getNumEdges() << "\n";
    //    std::cout << "Number of cover edges -- " << numInitCoverEdges << "\n";
    //
    //    std::cout << "Number of cover edges after pruning -- " << numCoverEdges << "\n";
    //    std::cout << "% pruned edges -- " << ((double)(numInitCoverEdges - numCoverEdges) / (double)numInitCoverEdges)
    //    * 100 << "\n"; std::cout << "Pruned by domination -- " << domPruned << "\n";
    //
    //    std::cout << "Ended k-PC construction" << std::endl;
}

bool CoverGraph::hasOutgoingEdges(int i) const {
    return outgoingCoverEdges.count(i) > 0;
}

bool CoverGraph::hasIncomingEdges(int i) const {
    return incomingCoverEdges.count(i) > 0;
}

const std::unordered_map<int, std::unordered_set<int>>& CoverGraph::getOutgoingEdges(int i) const {
    return outgoingCoverEdges.at(i);
}

const std::unordered_map<int, std::unordered_set<int>>& CoverGraph::getIncomingEdges(int i) const {
    return incomingCoverEdges.at(i);
}

std::vector<int> CoverGraph::getEdgeNodes(int i) const {
    return coverEdges[i].getInsideNodes();
}

std::vector<double> CoverGraph::linkPaths(const std::vector<double>& currentCriteria, const int numHops1,
                                          const std::vector<double>& suffixCriteria, const int numHops2) const {
    size_t numParameters = currentCriteria.size();
    std::vector<double> fullCriteria(numParameters, 0);
    fullCriteria[0] = currentCriteria[0] + suffixCriteria[0];

    for (int i = 1; i < numParameters - 1; ++i) {
        fullCriteria[i] =
            (currentCriteria[i] * static_cast<double>(numHops1)) +
            (suffixCriteria[i] * static_cast<double>(numHops2)) / static_cast<double>(numHops1 + numHops2);
    }
    fullCriteria.back() = currentCriteria.back() + suffixCriteria.back();

    // fullCriteria.back() = (currentCriteria.back() * static_cast<double>(numHops1)) +
    //                       (suffixCriteria.back() * static_cast<double>(numHops2)) /
    //                       static_cast<double>(numHops1 + numHops2);

    return fullCriteria;
}

bool CoverGraph::checkDomination(const std::vector<double>& vector1, const std::vector<double>& vector2) const {
    size_t numParameters = vector1.size();
    // for (int i = 0; i < numParameters; ++i) {
    //     if (vector1[i] > vector2[i])
    //         return false;
    // }
    // return true;
    if (vector1[0] > vector2[0]) {
        return false;
    }
    if (vector1[1] < vector2[1]) {
        return false;
    }
    if (vector1[2] < vector2[2]) {
        return false;
    }
    if (vector1[3] > vector2[3]) {
        return false;
    }
    return true;
}

int CoverGraph::getNumEdges() const {
    return numCoverEdges;
}

int CoverGraph::getNumNodes() const {
    return numCoverNodes;
}

// std::map<int, std::set<label *, lexCompare>> CoverGraph::tKpcMls(int source) {
//     auto t1 = std::chrono::high_resolution_clock::now();
//
//     if (coverNodes.count(source) == 0)
//         return {};
//
//     std::map<int, std::set<label*, lexCompare>> closedLabels;
//     std::map<int, std::set<label*, lexCompare>> openLabels;
//     std::map<int, std::set<std::vector<int>>> tSets;
//
//     int numParameters = 2;
//
////    std::stack<label*> unusedLabels;
//
//    std::set<label*, lexCompare> toExpand;
//    long numLabels = 0;
//    auto* sourceLabel = new label(source, std::vector<int>(numParameters, 0), nullptr, -1);
//    toExpand.insert(sourceLabel);
//    numLabels += 1;
//    openLabels[source].insert(sourceLabel);
//
//    while (!toExpand.empty()) {
//        label* currentLabel = *toExpand.begin();
//        toExpand.erase(toExpand.begin());
//        int currentNode = currentLabel->node;
//        std::vector<double> currentCriteria = currentLabel->parameters;
//
//        openLabels[currentNode].erase(currentLabel);
//        closedLabels[currentNode].insert(currentLabel);
//        updateTSet(tSets[currentNode], currentLabel);
//
//        for (int edge: outgoingCoverEdges[currentNode]) {
//            int nextNode = coverEdges[edge].getEndNode();
//            if (currentLabel->pParent && nextNode == currentLabel->pParent->node)
//                continue;
//            std::vector<double> suffixCriteria = buildSuffixParameters(edge);
//            std::vector<double> newCriteria = linkPaths(currentCriteria, suffixCriteria);
//
//            if (tDiscard(closedLabels[nextNode], tSets[nextNode], newCriteria))
//                continue;
//
//            bool isDominated = false;
//            std::stack<label *> toRemove;
//            for (auto &lab: openLabels[nextNode]) {
//                if (checkDomination(lab->parameters, newCriteria)) {
//                    isDominated = true;
//                    break;
//                } else if (checkDomination(newCriteria, lab->parameters)) {
//                    toRemove.push(lab);
//                }
//            }
//            if (isDominated)
//                continue;
//            while (!toRemove.empty()) {
//                auto *lab = toRemove.top();
//                toRemove.pop();
//                toExpand.erase(lab);
//                openLabels[nextNode].erase(lab);
////                unusedLabels.push(lab);
//                delete lab;
//                numLabels -= 1;
//            }
//
////            if (!unusedLabels.empty()) {
////                auto *newLabel = unusedLabels.top();
////                unusedLabels.pop();
////                newLabel->reset(nextNode, newCriteria, currentLabel, edge);
////                openLabels[nextNode].insert(newLabel);
////                numLabels += 1;
////                toExpand.insert(newLabel);
////            }
////            else {
//            auto *newLabel = new label(nextNode, newCriteria, currentLabel, edge);
//            openLabels[nextNode].insert(newLabel);
//            numLabels += 1;
//            toExpand.insert(newLabel);
////            }
//
//
//
//        }
//    }
//
////    while (!unusedLabels.empty()) {
////        auto *lab = unusedLabels.top();
////        unusedLabels.pop();
////        delete lab;
////    }
//
//    auto t2 = std::chrono::high_resolution_clock::now();
//    double duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
//    duration *= 1e-9;
////    cout << "K: " << pow(2, degree) << endl;
////    cout << "MLS duration: " << duration << endl;
////    cout << "Number of labels: " << numLabels << endl;
////    cout << "------------------------------------------------------------" << endl;
//    return closedLabels;
//}

//// Backward search tkPCMLS from the target node
// std::map<int, std::set<label *, lexCompare>> CoverGraph::backwardTKpcMLS(int target) {
//     auto t1 = std::chrono::high_resolution_clock::now();
//
//     if (coverNodes.count(target) == 0)
//         return {};
//
//     std::map<int, std::set<label*, lexCompare>> closedLabels;
//     std::map<int, std::set<label*, lexCompare>> openLabels;
//     std::map<int, std::set<std::vector<int>>> tSets;
//
//     std::set<label*, lexCompare> toExpand;
//     long numLabels = 0;
//     auto* targetLabel = new label(target, std::vector<int>(numParameters, 0), nullptr, -1);
//     toExpand.insert(targetLabel);
//     numLabels += 1;
//     openLabels[target].insert(targetLabel);
//
//     while (!toExpand.empty()) {
//         label *currentLabel = *toExpand.begin();
//         toExpand.erase(toExpand.begin());
//         int currentNode = currentLabel->node;
//         std::vector<int> currentCriteria = currentLabel->parameters;
//
//         openLabels[currentNode].erase(currentLabel);
//         closedLabels[currentNode].insert(currentLabel);
//         updateTSet(tSets[currentNode], currentLabel);
//
//         for (int edge: incomingCoverEdges[currentNode]) {
//             int nextNode = coverEdges[edge].getStartNode();
//             if (currentLabel->parent && nextNode == currentLabel->parent->node)
//                 continue;
//             std::vector<int> prefixCriteria = buildSuffixParameters(coverEdges[edge]);
//             std::vector<int> newCriteria = linkPaths(prefixCriteria, currentCriteria);
//
//             if (tDiscard(closedLabels[nextNode], tSets[nextNode], newCriteria))
//                 continue;
//
//             bool isDominated = false;
//             std::stack<label *> toRemove;
//             for (auto &lab: openLabels[nextNode]) {
//                 if (checkDomination(lab->parameters, newCriteria)) {
//                     isDominated = true;
//                     break;
//                 } else if (checkDomination(newCriteria, lab->parameters)) {
//                     toRemove.push(lab);
//                 }
//             }
//             if (isDominated)
//                 continue;
//             while (!toRemove.empty()) {
//                 auto *lab = toRemove.top();
//                 toRemove.pop();
//                 toExpand.erase(lab);
//                 openLabels[nextNode].erase(lab);
//                 delete lab;
//                 numLabels -= 1;
//             }
//
//             auto *newLabel = new label(nextNode, newCriteria, currentLabel, edge);
//             openLabels[nextNode].insert(newLabel);
//             numLabels += 1;
//             toExpand.insert(newLabel);
//         }
//     }
//
//     auto t2 = std::chrono::high_resolution_clock::now();
//     double duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
//     duration *= 1e-9;
//     return closedLabels;
// }

void CoverGraph::updateTSet(std::set<std::vector<double>>& tSet, label* lab) const {
    size_t numParameters = lab->parameters.size();
    std::vector<double> newVec(lab->parameters.begin() + 1, lab->parameters.end());
    for (const std::vector<double>& v : tSet) {
        bool isDominated = true;
        for (int i = 0; i < numParameters - 1; ++i) {
            if (v[i] > newVec[i]) {
                isDominated = false;
            }
        }
        if (isDominated) {
            return;
        }
    }

    auto inserted = tSet.insert(newVec);
    auto it = inserted.first;
    if (inserted.second) {
        it++;
        std::stack<typename std::set<std::vector<double>>::iterator> toRemove;
        while (it != tSet.end()) {
            bool isDominated = true;
            for (int i = 0; i < numParameters - 1; ++i) {
                if (newVec[i] > (*it)[i]) {
                    isDominated = false;
                }
            }
            if (isDominated) {
                toRemove.push(it);
            }
            it++;
        }

        while (!toRemove.empty()) {
            tSet.erase(toRemove.top());
            toRemove.pop();
        }
    }
}

bool CoverGraph::tDiscard(std::set<label*, lexCompare>& closedLabels, std::set<std::vector<double>>& tSet,
                          std::vector<double>& newVec) {
    // Check first parameter dominance
    if (closedLabels.empty()) {
        return false;
    }
    auto& maxP = *(closedLabels.rbegin());

    if (newVec[0] < maxP->parameters[0]) {
        return false;
    }

    return std::any_of(tSet.begin(), tSet.end(),
                       [&newVec, this](const auto& lab) { return checkTDomination(lab, newVec); });
}

bool CoverGraph::checkTDomination(const std::vector<double>& t, const std::vector<double>& v) const {
    size_t numParameters = v.size();
    for (int i = 0; i < numParameters - 1; ++i) {
        if (t[i] > v[i + 1])
            return false;
    }
    return true;
}

const CoverEdge& CoverGraph::getCoverEdge(int i) const {
    return coverEdges[i];
}

std::vector<int> CoverGraph::getEdgeEdges(int i) const {
    return coverEdges[i].getInsideEdges();
}

std::set<int> CoverGraph::getCoverNodes() const {
    return coverNodes;
}

const Graph& CoverGraph::getOriginalGraph() const {
    return originalGraph;
}

std::unordered_set<int> CoverGraph::getConnectedOutgoingEdges(int incomingOriginalEdge) const {
    std::unordered_set<int> connectedOriginalEdges = originalGraph.getEdge(incomingOriginalEdge).getOutgoingEdges();
    int endNode = originalGraph.getEdge(incomingOriginalEdge).getEndNode();
    std::unordered_set<int> connectedCoverEdges;

    auto it1 = outgoingCoverEdges.find(endNode);
    if (it1 != outgoingCoverEdges.end()) {
        for (int edge : connectedOriginalEdges) {
            // Check if the map contains a required entry
            auto it2 = it1->second.find(edge);
            if (it2 != it1->second.end()) {
                connectedCoverEdges.insert(it2->second.begin(), it2->second.end());
            }
        }
    }
    return connectedCoverEdges;
}

std::unordered_set<int> CoverGraph::getConnectedIncomingEdges(int outgoingOriginalEdge) const {
    std::unordered_set<int> connectedOriginalEdges = originalGraph.getEdge(outgoingOriginalEdge).getIncomingEdges();
    int startNode = originalGraph.getEdge(outgoingOriginalEdge).getStartNode();
    std::unordered_set<int> connectedCoverEdges;
    auto it1 = incomingCoverEdges.find(startNode);
    if (it1 != incomingCoverEdges.end()) {
        for (int edge : connectedOriginalEdges) {
            // Check if the map contains a required entry
            auto it2 = it1->second.find(edge);
            if (it2 != it1->second.end()) {
                connectedCoverEdges.insert(it2->second.begin(), it2->second.end());
            }
        }
    }
    return connectedCoverEdges;
}

int CoverGraph::getClusterID() const {
    return clusterID;
}

void CoverGraph::setClusterID(int id) {
    clusterID = id;
}

std::shared_ptr<const CoverGraph> CoverGraph::getPrecedingCoverGraph() const {
    return precedingCoverGraph;
}

const CoverEdge& CoverGraph::originalToCoverEdge(int originalEdge) const {
    return coverEdges[originalToCoverEdges.at(originalEdge)];
}

const std::vector<CoverEdge>& CoverGraph::getCoverEdges() const {
    return coverEdges;
}
