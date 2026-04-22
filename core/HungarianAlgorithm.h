#pragma once

#include <vector>

namespace mission {

class HungarianAlgorithm {
public:
    static std::vector<int> solve(const std::vector<std::vector<double>>& costMatrix);
};

}  // namespace mission
