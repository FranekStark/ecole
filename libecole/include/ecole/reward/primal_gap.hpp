#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>
#include <unordered_set>

#include "ecole/reward/abstract.hpp"
#include "ecole/scip/model.hpp"

namespace ecole::reward {


class ConfinedPrimalGapIntegral : public RewardFunction {
public:
    ConfinedPrimalGapIntegral(const std::function<double(std::string)> & primal_bound_lookup_fun, double time_limit, double importance, std::string name) noexcept;
	void before_reset(scip::Model& model) override;
	Reward extract(scip::Model& model, bool done = false) override;
    void incubent_event(SCIP* scip, SCIP_SOL* sol);
    static double calc_primal_gap(double primal_bound, double primal_val);
private:
    std::function<double(std::string)> primal_bound_lookup_fun_;
    double last_time_;
    double last_primal_gap_;
    double confined_primal_gap_integral_;
    double alpha_;
    int n_sols_;
    double primal_bound_;
    std::string name_;
    double time_limit_;
    double last_best_;

    /** EventHDRL */
    SCIP_EVENTHDLR* eventhdlr_;
    int filter_pos_;
    
    static
    SCIP_DECL_EVENTEXEC(eventExec);
    static
    SCIP_DECL_EVENTINIT(eventInit);
    static constexpr bool DEBUG_PRINTS = false;
    
};


}  // namespace ecole::reward
