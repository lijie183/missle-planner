#include "core/GeneticAllocator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace mission {

class GeneticAllocatorImpl {
public:
    GeneticAllocatorImpl(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& priorities,
        const GeneticAllocator::Options& options,
        std::mt19937& rng)
        : m_cost(costMatrix)
        , m_priorities(priorities)
        , m_opts(options)
        , m_rng(rng)
        , m_nMissiles(static_cast<int>(costMatrix.size()))
        , m_nTargets(static_cast<int>(costMatrix.empty() ? 0 : costMatrix[0].size())) {}

    std::vector<int> run() {
        if (m_nMissiles == 0 || m_nTargets == 0) {
            return std::vector<int>(m_nMissiles, -1);
        }

        initPopulation();
        evaluateAll();

        for (int gen = 0; gen < m_opts.generations; ++gen) {
            std::vector<std::vector<int>> nextPop;
            nextPop.reserve(static_cast<std::size_t>(m_opts.populationSize));

            std::sort(m_fitness.begin(), m_fitness.end(), [](const auto& a, const auto& b) {
                return a.second > b.second;
            });

            int eliteCount = std::max(2, m_opts.populationSize / 10);
            for (int i = 0; i < eliteCount && i < static_cast<int>(m_pop.size()); ++i) {
                nextPop.push_back(m_pop[m_fitness[i].first]);
            }

            while (static_cast<int>(nextPop.size()) < m_opts.populationSize) {
                int p1 = tournamentSelect();
                int p2 = tournamentSelect();
                auto child = crossover(m_pop[p1], m_pop[p2]);
                mutate(child);
                nextPop.push_back(std::move(child));
            }

            m_pop = std::move(nextPop);
            evaluateAll();
        }

        int bestIdx = 0;
        double bestFit = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(m_fitness.size()); ++i) {
            if (m_fitness[i].second > bestFit) {
                bestFit = m_fitness[i].second;
                bestIdx = m_fitness[i].first;
            }
        }

        return repairAssignment(m_pop[bestIdx]);
    }

private:
    void initPopulation() {
        m_pop.resize(static_cast<std::size_t>(m_opts.populationSize));
        for (auto& indiv : m_pop) {
            indiv.resize(static_cast<std::size_t>(m_nMissiles));
            std::vector<int> targets(m_nTargets);
            for (int i = 0; i < m_nTargets; ++i) {
                targets[i] = i;
            }
            std::shuffle(targets.begin(), targets.end(), m_rng);

            for (int i = 0; i < m_nMissiles; ++i) {
                if (i < m_nTargets) {
                    indiv[i] = targets[i];
                } else {
                    indiv[i] = -1;
                }
            }
            std::shuffle(indiv.begin(), indiv.end(), m_rng);
        }
    }

    double fitness(const std::vector<int>& indiv) const {
        double totalCost = 0.0;
        double priorityBonus = 0.0;
        int assigned = 0;

        std::vector<bool> targetUsed(m_nTargets, false);
        for (int i = 0; i < m_nMissiles; ++i) {
            int t = indiv[i];
            if (t < 0 || t >= m_nTargets) {
                continue;
            }
            if (targetUsed[t]) {
                totalCost += 1e12;
                continue;
            }
            targetUsed[t] = true;
            totalCost += m_cost[i][t];
            priorityBonus += static_cast<double>(m_priorities[t]) * 1000.0;
            ++assigned;
        }

        return priorityBonus - totalCost + assigned * 5000.0;
    }

    void evaluateAll() {
        m_fitness.clear();
        m_fitness.reserve(m_pop.size());
        for (int i = 0; i < static_cast<int>(m_pop.size()); ++i) {
            m_fitness.emplace_back(i, fitness(m_pop[i]));
        }
    }

    int tournamentSelect() const {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(m_pop.size()) - 1);
        int best = dist(m_rng);
        double bestFit = fitness(m_pop[best]);

        for (int k = 1; k < m_opts.tournamentSize; ++k) {
            int cand = dist(m_rng);
            double candFit = fitness(m_pop[cand]);
            if (candFit > bestFit) {
                best = cand;
                bestFit = candFit;
            }
        }
        return best;
    }

    std::vector<int> crossover(const std::vector<int>& p1, const std::vector<int>& p2) const {
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        std::vector<int> child(static_cast<std::size_t>(m_nMissiles));
        for (int i = 0; i < m_nMissiles; ++i) {
            child[i] = (prob(m_rng) < m_opts.crossoverRate) ? p1[i] : p2[i];
        }
        return child;
    }

    void mutate(std::vector<int>& indiv) const {
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        std::uniform_int_distribution<int> targetDist(-1, m_nTargets - 1);

        for (int i = 0; i < m_nMissiles; ++i) {
            if (prob(m_rng) < m_opts.mutationRate) {
                indiv[i] = targetDist(m_rng);
            }
        }
    }

    std::vector<int> repairAssignment(const std::vector<int>& indiv) const {
        std::vector<int> result(m_nMissiles, -1);
        std::vector<bool> targetUsed(m_nTargets, false);

        std::vector<std::pair<double, int>> candidates;
        candidates.reserve(static_cast<std::size_t>(m_nMissiles));
        for (int i = 0; i < m_nMissiles; ++i) {
            int t = indiv[i];
            if (t >= 0 && t < m_nTargets) {
                double pri = static_cast<double>(m_priorities[t]);
                candidates.emplace_back(-pri, i);
            }
        }
        std::sort(candidates.begin(), candidates.end());

        for (const auto& cand : candidates) {
            int i = cand.second;
            int t = indiv[i];
            if (t >= 0 && t < m_nTargets && !targetUsed[t]) {
                result[i] = t;
                targetUsed[t] = true;
            }
        }

        for (int i = 0; i < m_nMissiles; ++i) {
            if (result[i] >= 0) {
                continue;
            }
            double bestCost = std::numeric_limits<double>::infinity();
            int bestTarget = -1;
            for (int t = 0; t < m_nTargets; ++t) {
                if (targetUsed[t]) {
                    continue;
                }
                if (m_cost[i][t] < bestCost) {
                    bestCost = m_cost[i][t];
                    bestTarget = t;
                }
            }
            if (bestTarget >= 0) {
                result[i] = bestTarget;
                targetUsed[bestTarget] = true;
            }
        }

        return result;
    }

    const std::vector<std::vector<double>>& m_cost;
    const std::vector<int>& m_priorities;
    const GeneticAllocator::Options m_opts;
    std::mt19937& m_rng;
    int m_nMissiles;
    int m_nTargets;
    std::vector<std::vector<int>> m_pop;
    std::vector<std::pair<int, double>> m_fitness;
};

std::vector<int> GeneticAllocator::solve(
    const std::vector<std::vector<double>>& costMatrix,
    const std::vector<int>& targetPriorities,
    const Options& options) {
    std::mt19937 rng(42);
    GeneticAllocatorImpl impl(costMatrix, targetPriorities, options, rng);
    return impl.run();
}

}  // namespace mission
