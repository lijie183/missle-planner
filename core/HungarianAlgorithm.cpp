#include "core/HungarianAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mission {

std::vector<int> HungarianAlgorithm::solve(const std::vector<std::vector<double>>& costMatrix) {
    const int rows = static_cast<int>(costMatrix.size());
    if (rows == 0) {
        return {};
    }
    const int cols = static_cast<int>(costMatrix[0].size());
    if (cols == 0) {
        return std::vector<int>(rows, -1);
    }

    const int n = std::max(rows, cols);

    std::vector<std::vector<double>> a(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            a[i][j] = costMatrix[i][j];
        }
    }

    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(n + 1, 0.0);
    std::vector<int> p(n + 1, 0);
    std::vector<int> way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(n + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0];
            int j1 = 0;
            double delta = std::numeric_limits<double>::infinity();

            for (int j = 1; j <= n; ++j) {
                if (used[j]) {
                    continue;
                }
                const double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> assignment(rows, -1);
    for (int j = 1; j <= n; ++j) {
        if (p[j] >= 1 && p[j] <= rows && j >= 1 && j <= cols) {
            assignment[p[j] - 1] = j - 1;
        }
    }

    return assignment;
}

}  // namespace mission
