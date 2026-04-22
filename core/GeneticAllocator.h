#pragma once

#include <vector>

namespace mission {

class GeneticAllocator {
public:
    struct Options {
        int populationSize;
        int generations;
        double crossoverRate;
        double mutationRate;
        int tournamentSize;

        Options()
            : populationSize(80)
            , generations(200)
            , crossoverRate(0.85)
            , mutationRate(0.15)
            , tournamentSize(4) {}
    };

    static std::vector<int> solve(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& targetPriorities,
        const Options& options = Options());
};

}  // namespace mission
