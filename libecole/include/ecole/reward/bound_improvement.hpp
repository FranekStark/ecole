#pragma once

#include <functional>
#include <string>

#include "ecole/reward/abstract.hpp"
namespace ecole::reward {

class BoundImprovement : public RewardFunction {
public:

	BoundImprovement(const std::function<double(std::string)> & primal_bound_lookup_fun, double time_limit, double importance, std::string name) 
	void before_reset(scip::Model& model) override;
	Reward extract(scip::Model& model, bool done = false) override;

private:
    std::function<double(std::string)> primal_bound_lookup_fun_;
    double primal_bound_;
    std::string name_;
    double time_limit_;
    double last_best_;
    SCIP_EVENTHDLR* eventhdlr_;
    int filter_pos_;
    
    static
    SCIP_DECL_EVENTEXEC(eventExec);
    static
    SCIP_DECL_EVENTINIT(eventInit);
};


}