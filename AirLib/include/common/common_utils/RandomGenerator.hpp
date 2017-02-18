// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef commn_utils_sincos_hpp
#define commn_utils_sincos_hpp

#include <random>

namespace common_utils {

template<typename TReturn, typename TDistribution, unsigned int Seed=42>
class RandomGenerator {
  public:
    template<typename... DistArgs>
    RandomGenerator(DistArgs... dist_args)
        : dist_(dist_args...), rand_(Seed) {
    }

    TReturn next() {
        return dist_(rand_);
    }

    void reset() {
        rand_.seed(Seed);
        dist_.reset();
    }

  private:
    TDistribution dist_;
    std::mt19937 rand_;
};

typedef RandomGenerator<double, std::uniform_real_distribution<double>> RandomGeneratorD;
typedef RandomGenerator<float, std::uniform_real_distribution<float>> RandomGeneratorF;
typedef RandomGenerator<int, std::uniform_int_distribution<int>> RandomGeneratorI;
typedef RandomGenerator<float, std::normal_distribution<float>> RandomGeneratorGaussianF;
typedef RandomGenerator<double, std::normal_distribution<double>> RandomGeneratorGaussianD;

}
#endif
