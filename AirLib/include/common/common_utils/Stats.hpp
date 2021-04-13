// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_Stats_hpp
#define common_utils_Stats_hpp

#include <algorithm>
#include <numeric>
#include <vector>

namespace common_utils {

    class Stats {
    public:
        Stats()
        {
            clear();
        }

        void clear()
        {
            values.clear();
        }

        void insert(double x)
        {
            values.push_back(x);
        }

        int64_t size() const
        {
            return values.size();
        }

        double mean() const
        {
            auto sz = size();
            if (sz == 0) {
                return 0;
            }
            return std::accumulate(values.begin(), values.end(), 0.0) / sz;
        }

        double min() const
        {            
            return *std::min_element(values.begin(), values.end());
        }

        double max() const
        {
            return *std::max_element(values.begin(), values.end());
        }

        double variance() const
        {
            auto sz = this->size();
            auto mean = this->mean();
            auto variance_func = [&mean, &sz](double accumulator, const double& val) {
                return accumulator + ((val - mean) * (val - mean) / (sz - 1));
            };

            return std::accumulate(values.begin(), values.end(), 0.0, variance_func);
        }

        double standardDeviation() const
        {
            return sqrt(variance());
        }

    private:
        std::vector<double> values;

    }; //class

}  //namespace
#endif
