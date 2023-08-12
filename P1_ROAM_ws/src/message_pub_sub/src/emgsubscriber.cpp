#include <functional>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "pub_sub/msg/emgdata.hpp"

class EmgSubscriber : public rclcpp::Node
{
public:
    EmgSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<pub_sub::msg::Emgdata>(
            "simulated_emg", 10, std::bind(&EmgSubscriber::emg_callback, this, std::placeholders::_1));
    }

private:
    void emg_callback(const pub_sub::msg::Emgdata::SharedPtr msg)
    {
        // std::cout << "Received EMG data: ";
        // for (const auto &value_ : msg->emg_signal)
        // {
        //     std::cout << value_ << " ";
        // }
        // std::cout << std::endl;

        update_window(msg->emg_signal);

        std::vector<float> smoothed_emg = compute_average();

        std::cout << "Smoothed EMG data: ";
        for (const auto &value_ : smoothed_emg)
        {
            std::cout << value_ << " ";
        }
        std::cout << std::endl;
    }

    void update_window(const std::array<long int, 8>& new_data)
    {
        if (sliding_window_.size() == WINDOW_SIZE)
        {
            std::array<long int, 8>& front = sliding_window_.front();
            for (size_t i = 0; i < front.size(); i++)
            {
                curr_sum_channel[i] -= front[i];
            }
            sliding_window_.pop_front();
        }

        sliding_window_.push_back(new_data);
        for (size_t i = 0; i < new_data.size(); i++)
        {
            curr_sum_channel[i] += new_data[i];
        }
    }

    std::vector<float> compute_average()
    {
        std::vector<float> average(8, 0);
        int current_size = sliding_window_.size();
        for (size_t i = 0; i < average.size(); i++)
        {
            average[i] = curr_sum_channel[i] / current_size;
        }
        return average;
    }

    rclcpp::Subscription<pub_sub::msg::Emgdata>::SharedPtr subscription_;
    std::deque<std::array<long int, 8>> sliding_window_;
    std::vector<int> curr_sum_channel = std::vector<int>(8, 0);
    const size_t WINDOW_SIZE = 20;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmgSubscriber>());
    rclcpp::shutdown();
    return 0;
}
