/*#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msg/int.msg"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_1 = this->create_subscription<std_msgs::msg::String>(
      "start_and_mode", 1, std::bind(&MinimalSubscriber::start_and_mode, this, _1));

      subscription_2 = this->create_subscription<std_msgs::msg::Bool>(
      "start", 1, std::bind(&MinimalSubscriber::start, this, _1));

      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    bool mod_haut = 0;
    bool mod_bas = 0;
    bool drill_haut = 0;
    bool drill_bas = 0;
    const char* mode = 0;
    float step = 0;

    void start(const std_msgs::msg::Bool::SharedPtr msg)
    {

    }
    


    void start_and_mode(const std_msgs::msg::String::SharedPtr msg)
    {
      step = 1;
      mode = msg->data.c_str();
      if (!mod_haut)
      {
        //appel montée
      }

      if (!drill_haut)
      {
        //appel montée
      }
      //appel descente module
      step = 2;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"     // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}