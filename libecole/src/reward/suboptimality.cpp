#include "ecole/reward/suboptimality.hpp"
#include "ecole/reward/primal_gap.hpp"

namespace ecole::reward {

SubOptimality::SubOptimality(double time_limit, const std::function<double(std::string)> & primal_bound_lookup_fun) : 
time_limit_(time_limit),
primal_bound_lookup_fun_(primal_bound_lookup_fun)
{

}

void SubOptimality::before_reset(scip::Model& model) {
	primal_bound_ = primal_bound_lookup_fun_(model.name());
}

Reward SubOptimality::extract(scip::Model& model, bool /* done */) {
    int n_sols = SCIPgetNSols(model.get_scip_ptr());
    SCIP_SOL** sols_arr_ptr = SCIPgetSols(model.get_scip_ptr());
    SCIP_OBJSENSE objective = SCIPgetObjsense(model.get_scip_ptr());
    if(SCIPgetStage(model.get_scip_ptr()) == SCIP_STAGE_PROBLEM){
        return 1.0;
    }
    double curren_best_objective;
    if(objective == SCIP_OBJSENSE::SCIP_OBJSENSE_MAXIMIZE){
        curren_best_objective = std::numeric_limits<double>::min();
    }else if(objective == SCIP_OBJSENSE::SCIP_OBJSENSE_MINIMIZE ){
        curren_best_objective = std::numeric_limits<double>::max();
    }
    SCIP_SOL* current_best_suboptimal_sol = nullptr;
    for(int sol_idx = 0; sol_idx < n_sols; sol_idx++){
        SCIP_SOL* sol = *(sols_arr_ptr + sol_idx);
        double sol_time = SCIPgetSolTime(model.get_scip_ptr(), sol);
        double sol_obj = SCIPgetSolOrigObj(model.get_scip_ptr(), sol);
        if((sol_time < time_limit_)
        && (       ((objective == SCIP_OBJSENSE::SCIP_OBJSENSE_MAXIMIZE) && (sol_obj > curren_best_objective))
                || ((objective == SCIP_OBJSENSE::SCIP_OBJSENSE_MINIMIZE) && (sol_obj < curren_best_objective)))){
            current_highest_time_below_limit = sol_time;
            curren_best_objective = sol_obj;
            current_best_suboptimal_sol = sol;
        }   
    }
    if(current_best_suboptimal_sol == nullptr){
        return 1.0;
    }
	
	return ConfinedPrimalGapIntegral::calc_primal_gap(primal_bound_, curren_best_objective);
}

}  // namespace ecole::reward
