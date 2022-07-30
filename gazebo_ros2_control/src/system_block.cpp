#include "gazebo_ros2_control/system_block.hpp"

#include <stdexcept>

namespace gazebo_ros2_control {

SystemBlock::SystemBlock(std::deque<double> num, std::deque<double> den) {
    num_ = num;
    den_ = den;

    for (long unsigned int i = 0; i < num.size(); i++) {
        u_passed_.push_back(0);
    }

    for (long unsigned int i = 0; i < den.size() - 1; i++) {
        y_passed_.push_back(0);
        den_cropped_.push_back(den_[i + 1]);
    }
}

void SystemBlock::setup(std::deque<double> num, std::deque<double> den) {
    num_ = num;
    den_ = den;

    for (long unsigned int i = 0; i < num.size(); i++) {
        u_passed_.push_back(0);
    }

    for (long unsigned int i = 0; i < den.size() - 1; i++) {
        y_passed_.push_back(0);
        den_cropped_.push_back(den_[i + 1]);
    }
}

double SystemBlock::get_output(double u) {
    u_passed_.push_front(u);
    u_passed_.pop_back();

    double y = dot_prod(num_, u_passed_) - dot_prod(den_cropped_, y_passed_);

    y_passed_.push_front(y);
    y_passed_.pop_back();

    return y;
}

double SystemBlock::dot_prod(std::deque<double> x, std::deque<double> y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x and y must be the same size");
    }

    double p = 0;
    for (long unsigned int i = 0; i < x.size(); i++) {
        p += x[i] * y[i];
    }
    return p;
}

}  // namespace gazebo_ros2_control