#include "ecole/observation/treerecorder.hpp"
#include "scip/scipdefplugins.h"
#include <iostream>

namespace ecole::observation{

void TreeRecorder::before_reset(scip::Model& model) {

	SCIP_CALL_ABORT(SCIPincludeEventhdlrBasic(
		model.get_scip_ptr(),
		&eventhdlr_,
		"TreeRecorder",
		"Records the BnB tree structure",
		&TreeRecorder::eventExec,
		reinterpret_cast<SCIP_EVENTHDLRDATA*>(this)));
	SCIP_CALL_ABORT(SCIPsetEventhdlrInit(model.get_scip_ptr(), eventhdlr_, &TreeRecorder::eventInit));
}

TreeRecorderObs TreeRecorder::extract(scip::Model& model, bool done){
	if(done && (eventhdlr_ != NULL)){
        SCIPdropEvent(model.get_scip_ptr(), SCIP_EVENTTYPE_NODEEVENT | SCIP_EVENTTYPE_SOLEVENT,  eventhdlr_, NULL, -1);
		eventhdlr_ = NULL;
	}
	return TreeRecorderObs(SCIPnodeGetNumber(SCIPgetFocusNode(model.get_scip_ptr())), depth_groups_, nodes_);
} 

TreeRecorderObs::TreeRecorderObs(long long node_id, std::unordered_map<int, std::vector<long long>>& depth_groups, std::unordered_map<long long, Node>& nodes):
node_id_(node_id),
depth_groups_(depth_groups),
nodes_(nodes)
{}

SCIP_DECL_EVENTINIT(TreeRecorder::eventInit) {
	SCIP_CALL(SCIPcatchEvent(scip, SCIP_EVENTTYPE_NODEEVENT, eventhdlr, NULL, NULL));
	SCIP_CALL(SCIPcatchEvent(scip, SCIP_EVENTTYPE_SOLEVENT, eventhdlr, NULL, NULL));
	return SCIP_OKAY;
}

SCIP_DECL_EVENTEXEC(TreeRecorder::eventExec) {
	TreeRecorder* self = reinterpret_cast<TreeRecorder*>(SCIPeventhdlrGetData(eventhdlr));
	SCIP_EVENTTYPE eventtype = SCIPeventGetType(event);

	if (eventtype == SCIP_EVENTTYPE_NODEFOCUSED) {
		SCIP_NODE* node = SCIPeventGetNode(event);
		// Add the node
		self->addNode(scip, node);
	} else if (eventtype == SCIP_EVENTTYPE_NODEBRANCHED) {
		SCIP_NODE* node = SCIPeventGetNode(event);
		self->markLeaf(scip, node, false);
	} else if (eventtype == SCIP_EVENTTYPE_NODEFEASIBLE || eventtype == SCIP_EVENTTYPE_NODEINFEASIBLE) {
		SCIP_NODE* node = SCIPeventGetNode(event);
		self->markLeaf(scip, node, true);
	} else if(eventtype == SCIP_EVENTTYPE_POORSOLFOUND || eventtype == SCIP_EVENTTYPE_BESTSOLFOUND){
		SCIP_SOL* sol = SCIPeventGetSol(event);
		self->addSol(scip, sol);
	}

	return SCIP_OKAY;
}

void TreeRecorder::addNode(SCIP* scip, SCIP_NODE* node) {
	double time = SCIPgetSolvingTime(scip);
	long long id = SCIPnodeGetNumber(node);
	int depth = SCIPnodeGetDepth(node);
	depth_groups_[depth].push_back(id);
	long long parent_id;
	auto parent = SCIPnodeGetParent(node);
	if (parent) {
		parent_id = SCIPnodeGetNumber(parent);
		nodes_[parent_id].children_ids_.push_back(id);
	} else {
		parent_id = -1;
	}

	nodes_[id] = Node{ id, parent_id, {}, false, depth, time, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
	//std::cout << "Add node node_id: " << id << std::endl;
}

void TreeRecorder::markLeaf(SCIP* scip, SCIP_NODE* node, bool leaf) {
	double time = SCIPgetSolvingTime(scip);
	long long node_id = SCIPnodeGetNumber(node);
	nodes_[node_id].isLeaf = leaf;
	nodes_[node_id].end_time = time;
	auto best_sol = SCIPgetBestSol(scip);
	if(best_sol){
		nodes_[node_id].primal_obj = SCIPgetSolOrigObj(scip, best_sol);
	}
	//std::cout << "Mark leaf node_id: " << node_id << " - " << leaf << std::endl;
}


void TreeRecorder::addSol(SCIP* scip, SCIP_SOL* sol) {
	double primal_obj = SCIPgetSolOrigObj(scip, sol);
	double time = SCIPgetSolTime(scip, sol);
	long long node_id = SCIPgetSolNodenum(scip, sol);
	nodes_[node_id].sols.push_back({primal_obj, time, node_id});
	//std::cout << "Sol on node_id: " << node_id << std::endl;
}


double TreeRecorderObs::GetSubTreeConfinedPrimalGapIntegral(double primal_obj_bound, double time_limit, double importance, bool assume_no_sol_before) {
	double alpha = time_limit / log(importance);

	double last_incubent_time;
	double last_incubent_obj;
	if(nodes_[node_id_].parent_id != -1){
		last_incubent_time = nodes_[nodes_[node_id_].parent_id].end_time;
		last_incubent_obj = nodes_[nodes_[node_id_].parent_id].primal_obj;
	}else{ // We are root node
		last_incubent_time = nodes_[node_id_].start_time;
		last_incubent_obj = nodes_[node_id_].primal_obj;
	}

	if(assume_no_sol_before){
		last_incubent_obj = std::numeric_limits<double>::infinity();
	}

	double current_time = last_incubent_time;
	double confined_sub_integral = 0;

	std::list<long long> node_backlog;
	node_backlog.push_front(node_id_);
	while (!node_backlog.empty()) {
		// select first node in backlog (the first one)
		auto focus_node = node_backlog.front();
		for (auto& sol : nodes_[focus_node].sols) {
			if (sol.primal_obj_ < last_incubent_obj) {
				double last_primal_gap =
					reward::ConfinedPrimalGapIntegral::calc_primal_gap(primal_obj_bound, last_incubent_obj);
				double sol_offset_time = sol.time_ - nodes_[focus_node].start_time;
				confined_sub_integral += alpha * last_primal_gap *
																 (exp((current_time + sol_offset_time) / alpha) - exp(last_incubent_time / alpha));
				assert(sol.time_ > nodes_[focus_node].start_time);
				last_incubent_obj = sol.primal_obj_;
				last_incubent_time = current_time + sol_offset_time;
			}
		}
		// end of focues node, set current time and proceed
		current_time += (nodes_[focus_node].end_time - nodes_[focus_node].start_time);
		assert(nodes_[focus_node].children_ids_.size() <= 2);
		node_backlog.pop_front();  // Remove myself
		// emplace the childern nodes based on timing
		for (auto children_id : nodes_[focus_node].children_ids_) {
			if (node_backlog.empty()) {
				node_backlog.push_front(children_id);
			} else {
				auto back_log_item = node_backlog.end();
				// Assuming the children are later then the current backlog, we start at the end
				for (; back_log_item != node_backlog.begin(); back_log_item--) {
					if (nodes_[children_id].start_time >= nodes_[*(std::prev(back_log_item))].start_time) {
						break;
					}
				}
				node_backlog.insert(back_log_item, children_id);
			}
		}
	}
	// extrapolate the intgegral if time limit not hit
	if (last_incubent_time < time_limit) {
		double last_primal_gap = reward::ConfinedPrimalGapIntegral::calc_primal_gap(primal_obj_bound, last_incubent_obj);
		confined_sub_integral += alpha * last_primal_gap *
														 (exp((last_incubent_time + time_limit) / alpha) - exp(last_incubent_time / alpha));
	}

	return confined_sub_integral;
}
}