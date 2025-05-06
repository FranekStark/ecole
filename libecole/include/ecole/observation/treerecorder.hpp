#ifndef TREERECORDER_H
#define TREERECORDER_H

#include "scip/scip.h"
#include "scip/type_event.h"

#include <unordered_map>
#include <vector>
#include <unordered_set>

struct TreeRecorderObs
{
    long long node;
    long long parent;
    bool isLeaf;
    int depth;
    double lowerbound;
    double estimate;
};

class TreeRecorder
{
public:
    explicit TreeRecorder(SCIP* scip);
    SCIP_RETCODE registerEvents();

private:
    static
    SCIP_DECL_EVENTEXEC(eventExecTreeRecorder);

    void addNode(SCIP_NODE* node);
    void markLeaf(SCIP_NODE* node);

    SCIP* scip_;
    SCIP_EVENTHDLR* eventhdlr_;

    std::unordered_set<long long> leaf_nodes_;
    std::unordered_map<long long, std::vector<long long>> children_;
    std::unordered_map<long long, std::vector<long long>> depth_groups_;
    std::unordered_map<long long, long long> parents_;
};

#endif // TREERECORDER_H
