#include "ecole/observation/treerecorder.hpp"
#include "scip/scipdefplugins.h"
#include <iostream>

TreeRecorder::TreeRecorder(SCIP* scip)
: scip_(scip)
{
    SCIP_CALL_ABORT(SCIPincludeEventhdlrBasic(
        scip_, &eventhdlr_, "TreeRecorder",
        "Records the BnB tree structure",
        eventExecTreeRecorder, nullptr, this));
}

SCIP_RETCODE TreeRecorder::registerEvents()
{
    SCIP_CALL(SCIPcatchEvent(
        scip_,
        SCIP_EVENTTYPE_NODEEVENT,
        eventhdlr_, nullptr));
    return SCIP_OKAY;
}

SCIP_DECL_EVENTEXEC(TreeRecorder::eventExecTreeRecorder)
{
    TreeRecorder* self = static_cast<TreeRecorder*>(eventdata);
    SCIP_EVENTTYPE eventtype = SCIPeventGetType(event);

    if (eventtype == SCIP_EVENTTYPE_NODEFOCUSED)
    {
        SCIP_NODE* node = SCIP_CALL(SCIPeventGetNode(event))
        self->addNode(node);
    }
    else if (eventtype == SCIP_EVENTTYPE_NODEBRANCHED)
    {
        SCIP_NODE* node = SCIP_CALL(SCIPeventGetNode(event);
        // Nothing
    }
    else if (eventtype == SCIP_EVENTTYPE_NODEFEASIBLE ||
             eventtype == SCIP_EVENTTYPE_NODEINFEASIBLE)
    {
        SCIP_NODE* node = SCIP_CALL(SCIPeventGetNode(event));
        self->markLeaf(node);
    }

    return SCIP_OKAY;
}

void TreeRecorder::addNode(SCIP_NODE* node)
{
    if (nodes_.find(node) != nodes_.end())
        return; // Already known

    SCIP_NODE* parent = SCIP_CALL(SCIPnodeGetParent(node));
    long long id = SCIP_CALL(SCIPnodeGetNumber(node));
    
    nodes_[id]
    depth_groups_[id] = node.depth;

    if (parent) {
        long long parent_id = SCIP_CALL(SCIPnodeGetNumber(parent));
        children_[parent].push_back(id);
        parents_[id] = parent_id;
    }else{
        parents_[id] = -1;
    }

}

void TreeRecorder::markLeaf(SCIP_NODE* node)
{
    leaf_nodes_.insert(SCIP_CALL(SCIPnodeGetNumber(node)));
}

