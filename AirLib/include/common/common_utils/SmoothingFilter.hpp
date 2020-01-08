// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_SmoothingFilter_hpp
#define common_utils_SmoothingFilter_hpp

#include <vector>
#include <algorithm>
#include <tuple>

namespace common_utils {

// SmoothingFilter is a filter that removes the outliers that fall outside a given percentage from the min/max
// range of numbers over a given window size.  
template <typename T>
class SmoothingFilter {
    private:
        std::vector<T> buffer_;
        int window_size_;
        int window_size_2x_;
        float outlier_factor_;
        int buffer_index_;
    public:
        SmoothingFilter();
        SmoothingFilter(int window_size, float outlier_factor);
        void initialize(int window_size, float outlier_factor);
        
        // Each call to the filter method adds to the current window, and returns
        // the mean and variance over that window, after removing any outliers.
        // It returns an error variance = -1 until the window is full.  So you must
        // ignore the first window_size values returned by this filter.
        std::tuple<double,double> filter(T value);
};

template <typename T>
void SmoothingFilter<T>::initialize(int window_size, float outlier_factor) {
    buffer_.resize(window_size);
    window_size_ = window_size;
    window_size_2x_ = window_size * 2;
    outlier_factor_ = outlier_factor;
    buffer_index_ = 0;
}

template <typename T>
SmoothingFilter<T>::SmoothingFilter() {
    initialize(static_cast<int>(buffer_.capacity()), std::numeric_limits<float>::infinity());
}

template <typename T>
SmoothingFilter<T>::SmoothingFilter(int window_size, float outlier_factor) {
    initialize(window_size,  std::numeric_limits<float>::infinity());
}

template <typename T>
std::tuple<double,double> SmoothingFilter<T>::filter(T value){
    buffer_[buffer_index_++ % window_size_] = value;
    if (buffer_index_ == window_size_2x_)
        buffer_index_ = window_size_;    
    
    if (buffer_index_ >= window_size_) {
        // find min/max and range of the current buffer.
        double min = *std::min_element(buffer_.begin(), buffer_.end());
        double max = *std::max_element(buffer_.begin(), buffer_.end());
        auto range = (max - min);

        // outliers fall outside this lower and upper bound. 
        auto lower_bound = min + (range * outlier_factor_);
        auto upper_bound = max - (range * outlier_factor_);
        
        double sum = 0; int count = 0;
        for(auto i = 0; i < window_size_; ++i) {
            if (buffer_[i] >= lower_bound && buffer_[i] <= upper_bound) {
                sum += buffer_[i];
                ++count;
            }
        }

        if (count == 0) 
        {
            // this can happen if the numbers are oddly clustered around the min/max values.
            return std::make_tuple(double(min + range / 2), -1);
        }
        double mean = sum / count;
        
        double std_dev_sum = 0;
        for(auto i = 0; i < window_size_; ++i) {
            if (buffer_[i] >= lower_bound && buffer_[i] <= upper_bound) {
                double diff = buffer_[i] - mean;
                std_dev_sum += diff*diff;
            }
        }
        double variance = std_dev_sum / count; 

        return std::make_tuple(mean, variance);
    }
    else {
        // window is not yet full, so return an error code informing the caller that the filter
        // is not yet ready.
        return std::make_tuple(0, -1);
    }
}


} //namespace
#endif
