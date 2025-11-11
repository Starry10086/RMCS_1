#pragma once

#include <mutex>
#include <cmath>

namespace rmcs_core::filter {
class RampFilter {
public:
    explicit RampFilter(double max_rate, double dt) 
        : current_value_(0.0), max_rate_(max_rate), dt_(dt) {}
    
    double update(double target) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        double max_change = max_rate_ * dt_;
        double error = target - current_value_;
        
        if (std::abs(error) <= max_change) {
            current_value_ = target;
        } else {
            current_value_ += (error > 0 ? max_change : -max_change);
        }
        
        return current_value_;
    }
    
    void reset(double value = 0.0) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_value_ = value;
    }
private:
    double current_value_;
    double max_rate_;  // 最大变化率（单位/秒）
    double dt_;        // 时间步长（秒）
    mutable std::mutex mutex_;
};
}