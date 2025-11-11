#pragma once

#include <mutex>
#include <cmath>

namespace rmcs_core::filter {
template<int N>
class MovingAverageFilter {
public:
    explicit MovingAverageFilter() : index_(0), sum_(0.0) {
        buffer_.fill(0.0);
    }                                                                     
    double update(double value) {
        std::lock_guard<std::mutex> lock(mutex_);//加锁，防止多线程访问
        
        sum_ -= buffer_[index_];//先把合中的旧值减去
        buffer_[index_] = value;//然后把新值覆盖到旧值的位置
        sum_ += value;//然后合加上新值
        index_ = (index_ + 1) % N;//然后移动到下一个位置,方便下次计算
        
        return sum_ / N;//然后返回平均值
    }
private:
    std::array<double, N> buffer_;
    int index_;
    double sum_;
    mutable std::mutex mutex_;
};
}