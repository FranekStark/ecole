#pragma once

#include "ecole/reward/abstract.hpp"
#include "ecole/scip/model.hpp"

namespace ecole::reward {

class SubOptimality : public RewardFunction {
public:
	void before_reset(scip::Model& model) override;
	Reward extract(scip::Model& model, bool done = false) override;
    SubOptimality(double time_limit);
private:
	double time_limit_;
};

}  // namespace ecole::reward
