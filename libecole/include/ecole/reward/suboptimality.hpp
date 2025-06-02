#pragma once

#include "ecole/reward/abstract.hpp"
#include "ecole/scip/model.hpp"

namespace ecole::reward {

class SubOptimality : public RewardFunction {
public:
	void before_reset(scip::Model& model) override;
	Reward extract(scip::Model& model, bool done = false) override;
    SubOptimality(double time_limit, const std::function<double(std::string)> & primal_bound_lookup_fun);
private:
	double time_limit_;
	const std::function<double(std::string)> primal_bound_lookup_fun_;
	double primal_bound_;
};

}  // namespace ecole::reward
