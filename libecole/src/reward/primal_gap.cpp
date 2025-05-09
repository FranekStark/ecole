#include "ecole/reward/primal_gap.hpp"
#include <iostream>

namespace ecole::reward {
ConfinedPrimalGapIntegral::ConfinedPrimalGapIntegral(const std::function<double(std::string)> & primal_bound_lookup_fun, double time_limit, double importance, std::string name) noexcept:
primal_bound_lookup_fun_(primal_bound_lookup_fun),
last_time_(0),
last_primal_gap_(1),
confined_primal_gap_integral_(0),
alpha_(time_limit / log(importance)),
n_sols_(0),
primal_bound_(std::numeric_limits<double>::infinity()),
name_(name),
time_limit_(time_limit),
eventhdlr_(NULL),
filter_pos_(0)
{
    
}

void ConfinedPrimalGapIntegral::before_reset(scip::Model& model){
    //look up primal bound for this model
    primal_bound_ = primal_bound_lookup_fun_(model.name());
    if(DEBUG_PRINTS){
    std::cout << "Looked up primal value " << primal_bound_ << std::endl;
    }
    // reset fields
    last_primal_gap_ = 1.0;
    last_time_ = 0.0;
    confined_primal_gap_integral_ = 0.0;
    n_sols_ = 0;
    last_best_ = std::numeric_limits<double>::infinity();
    if(DEBUG_PRINTS){
    std::cout << "SCIP in stage: " << SCIPgetStage(model.get_scip_ptr()) << std::endl;
    }
    // add incubent solution event handler
    SCIP_CALL_ABORT(SCIPincludeEventhdlrBasic(model.get_scip_ptr(), &eventhdlr_, (std::string("incubent event hdlr ") + name_).c_str(), "handler to catch incubent events", &ConfinedPrimalGapIntegral::eventExec,  reinterpret_cast<SCIP_EVENTHDLRDATA*>(this)));
    if(DEBUG_PRINTS){
    std::cout << "Included event handler eventhdlr_: " << eventhdlr_ << ", this: " << this << std::endl;
    }
    SCIP_CALL_ABORT(SCIPsetEventhdlrInit(model.get_scip_ptr(), eventhdlr_, &ConfinedPrimalGapIntegral::eventInit));
    if(DEBUG_PRINTS){
    std::cout << "Set event handler init " << eventhdlr_ << std::endl;
    }
}

Reward ConfinedPrimalGapIntegral::extract(scip::Model& model, bool done){
    
    if(done && (eventhdlr_ != NULL)){
        SCIPdropEvent(model.get_scip_ptr(), SCIP_EVENTTYPE_BESTSOLFOUND,  eventhdlr_, NULL, -1);
        if(DEBUG_PRINTS){
        std::cout << "###################Drop event from  eventhdlr_" <<  eventhdlr_ << std::endl;
        }
        eventhdlr_ = NULL;
    }

    if(DEBUG_PRINTS){
    double best_sol_obj;
    if(SCIPgetNSols(model.get_scip_ptr()) >= 1){
        auto best_sol = SCIPgetBestSol(model.get_scip_ptr());
        best_sol_obj = SCIPgetSolOrigObj(model.get_scip_ptr(), best_sol);
    }else{
        best_sol_obj = std::numeric_limits<double>::infinity();
    }
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!! last_best: " <<  last_best_ << ", best_sol_obj: " << best_sol_obj << "off best: " << primal_bound_ << std::endl;
    }

    if(done && (time_limit_ > last_time_)){
        if(DEBUG_PRINTS){
        std::cout << "done and time limit over" <<  eventhdlr_ << std::endl;
        }
        return confined_primal_gap_integral_ + alpha_ * last_primal_gap_ * (exp(time_limit_ / alpha_) - exp(last_time_ / alpha_));
    }else{
        return confined_primal_gap_integral_;    
    }
    
}


SCIP_DECL_EVENTEXEC(ConfinedPrimalGapIntegral::eventExec)
{
    if(DEBUG_PRINTS){
    std::cout << "eventExec scip: "  << scip << ", eventhdlr: "<< eventhdlr << ", event: " << event << ", eventdata: " << eventdata << std::endl;
    }
    SCIP_EVENTTYPE eventtype = SCIPeventGetType(event);
    auto self = reinterpret_cast<ConfinedPrimalGapIntegral*>(SCIPeventhdlrGetData(eventhdlr));
    if(DEBUG_PRINTS){
    std::cout << "eventhdlrdata: " << self << std::endl;
    }
    if( eventtype == SCIP_EVENTTYPE_BESTSOLFOUND){
        SCIP_SOL * sol = SCIPeventGetSol(event);
        self->incubent_event(scip, sol);
    }
    return SCIP_OKAY;
}

SCIP_DECL_EVENTINIT(ConfinedPrimalGapIntegral::eventInit)
{
    if(DEBUG_PRINTS){
    std::cout << "eventInit "  << scip << ", "<< eventhdlr << std::endl;
    }
    SCIP_CALL(SCIPcatchEvent(scip, SCIP_EVENTTYPE_BESTSOLFOUND, eventhdlr, NULL, NULL));
    if(DEBUG_PRINTS){
    std::cout << "Catching event " << std::endl;
    }
    return SCIP_OKAY;
}



void ConfinedPrimalGapIntegral::incubent_event(SCIP* scip, SCIP_SOL* sol){
    double primal_val = SCIPgetSolOrigObj(scip, sol);
    last_best_ = primal_val;
    double time = SCIPsolGetTime(sol);
    double primal_gap = calc_primal_gap(primal_bound_, primal_val);
    if(DEBUG_PRINTS){
    std::cout << "INcubent event primal val " << primal_val << " at " << time << std::endl;
    }
    confined_primal_gap_integral_ += alpha_ * last_primal_gap_ * (exp(time / alpha_) - exp(last_time_ / alpha_));
    last_time_ = time;
    last_primal_gap_ = primal_gap;    
}


double ConfinedPrimalGapIntegral::calc_primal_gap(double primal_bound, double primal_val){
    double primal_gap = 1;
    if(isinf(primal_val)){
        primal_gap = 1;
    }
    else if((primal_bound == 0) && (primal_val == 0)){
        primal_gap = 0;
    }else if ((primal_bound * primal_val) < 0)
    {
        primal_gap = 1;
    }else{
        primal_gap = abs(primal_val - primal_bound) / MAX(abs(primal_val), abs(primal_bound));
    }
    return primal_gap;
}
}