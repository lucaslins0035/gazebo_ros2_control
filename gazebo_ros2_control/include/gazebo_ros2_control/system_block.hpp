#include <deque>

namespace gazebo_ros2_control {

class SystemBlock {
   public:
    SystemBlock() {}
    SystemBlock(std::deque<double> num, std::deque<double> den);

    double get_output(double u);

    void setup(std::deque<double> num, std::deque<double> den);

        private : double dot_prod(std::deque<double> x, std::deque<double> y);

    std::deque<double> num_;
    std::deque<double> den_;
    std::deque<double> u_passed_;
    std::deque<double> y_passed_;
    std::deque<double> den_cropped_;
};

}  // namespace gazebo_ros2_control
