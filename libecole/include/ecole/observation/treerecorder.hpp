#pragma once

#include "ecole/observation/abstract.hpp"
#include "ecole/reward/primal_gap.hpp"
#include "scip/scip.h"
#include "scip/type_event.h"

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace ecole::observation {

struct Sol {
    double primal_obj_;
    double time_;
    long long node_id;
};

struct Node {
    long long node_id;
    long long parent_id;
    std::vector<long long> children_ids_;
    bool isLeaf;
    int depth;
    // values
    double start_time;
    double end_time;
    double primal_obj;
    std::vector<Sol> sols;
};

class TreeRecorderObs {
public:
    TreeRecorderObs(long long node_id, std::unordered_map<int, std::vector<long long>>& depth_groups, std::unordered_map<long long, Node>& nodes);
	double GetSubTreeConfinedPrimalGapIntegral(double primal_obj_bound, double time_limit, double importance, bool assume_no_sol_before);
    long long node_id_;
    std::unordered_map<int, std::vector<long long>>& depth_groups_;
    std::unordered_map<long long, Node>& nodes_;
};


class TreeRecorder : public ObservationFunction<TreeRecorderObs> {
public:
	static SCIP_DECL_EVENTEXEC(eventExec);
	static SCIP_DECL_EVENTINIT(eventInit);

	void addNode(SCIP* scip, SCIP_NODE* node);
	void markLeaf(SCIP* scip, SCIP_NODE* node, bool leaf);
	void addSol(SCIP* scip, SCIP_SOL* sol);

    void before_reset(scip::Model& model) override;
	TreeRecorderObs extract(scip::Model& model, bool done) override;

private:

	SCIP_EVENTHDLR* eventhdlr_;

	std::unordered_map<int, std::vector<long long>> depth_groups_;
	std::unordered_map<long long, Node> nodes_;
};

}  // namespace ecole::observation
