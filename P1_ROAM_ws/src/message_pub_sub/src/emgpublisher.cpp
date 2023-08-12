#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <sstream>  // for ostringstream

#include "rclcpp/rclcpp.hpp"
#include "pub_sub/msg/emgdata.hpp"

using namespace std::chrono_literals;

class EmgPublisher : public rclcpp::Node
{
  public:
    EmgPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<pub_sub::msg::Emgdata>("simulated_emg", 10);
      timer_ = this->create_wall_timer(
        10ms, std::bind(&EmgPublisher::publish_emg, this)); //Publishing data at rate 100hz
    }

  private:
    void publish_emg()
    {
      auto message = std::make_unique<pub_sub::msg::Emgdata>();
      std::ostringstream os; // To create a string representation of the array

      for (auto &value : message->emg_signal)
      {
        value = distribution_(generator_);
        os << value << "  ";
      }

      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", os.str().c_str());
      publisher_->publish(*message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pub_sub::msg::Emgdata>::SharedPtr publisher_;
    size_t count_;
    std::default_random_engine generator_;
    std::uniform_int_distribution<long int> distribution_{0, 500}; // Randomly sample\ing between 0 and 500
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmgPublisher>());
  rclcpp::shutdown();
  return 0;
}
