#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

std::vector<rcl_interfaces::msg::Log> m_list;
int m_numMessages = 10;
int m_curr = 0;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = m_list[m_curr];
      message.stamp = this->now();
      m_curr++;
      if (m_curr>=m_numMessages){
        m_curr = 0;
      }
      
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    //message = rcl_interfaces::msg::Log();
    m_list.resize(m_numMessages);

    m_list[0].name = "node1";
    m_list[0].msg = "message number A";
    m_list[0].level = rcl_interfaces::msg::Log::DEBUG;
    m_list[1].name = "node1";
    m_list[1].msg = "message number B";
    m_list[1].level = rcl_interfaces::msg::Log::INFO;
    m_list[2].name = "node2";
    m_list[2].msg = "message number C";
    m_list[2].level = rcl_interfaces::msg::Log::DEBUG;
    m_list[3].name = "node3";
    m_list[3].msg = "message number D";
    m_list[3].level = rcl_interfaces::msg::Log::FATAL;
    m_list[4].name = "node3";
    m_list[4].msg = "message number E";
    m_list[4].level = rcl_interfaces::msg::Log::DEBUG;
    m_list[5].name = "node1";
    m_list[5].msg = "message number F";
    m_list[5].level = rcl_interfaces::msg::Log::INFO;
    m_list[6].name = "node1";
    m_list[6].msg = "message number G";
    m_list[6].level = rcl_interfaces::msg::Log::DEBUG;
    m_list[7].name = "node2";
    m_list[7].msg = "message number H";
    m_list[7].level = rcl_interfaces::msg::Log::INFO;
    m_list[8].name = "node2";
    m_list[8].msg = "message number I";
    m_list[8].level = rcl_interfaces::msg::Log::DEBUG;
    m_list[9].name = "node3";
    m_list[9].msg = "message number L";
    m_list[9].level = rcl_interfaces::msg::Log::DEBUG;
    

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}